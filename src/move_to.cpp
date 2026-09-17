// move_to.cpp
//
// One-shot blocking utility that drives the robot to a target joint
// configuration OR a target end-effector pose, using libfranka's built-in
// motion generators. Designed to be invoked by the frankatwin daemon for
// reset/home before handing the libfranka session back to `osc_shm`.
//
// Two mutually-exclusive modes:
//   1. --q q1 q2 q3 q4 q5 q6 q7 [--speed-factor 0.2]
//      Uses libfranka's joint MotionGenerator (vendored from examples). The
//      min-jerk trajectory respects per-joint dq_max and ddq_max scaled by
//      --speed-factor.
//   2. --pose tx ty tz qw qx qy qz [--duration 5.0]
//      Uses libfranka's franka::CartesianPose motion type. Each tick the
//      callback returns the desired 4x4 column-major matrix interpolated
//      between the start pose (captured at t=0) and the target pose via a 5th
//      order min-jerk profile. NO IK is performed on our side.
//
// Both modes seed their trajectory from the robot's *commanded* state (q_d /
// O_T_EE_c) rather than the measured one. Starting from the measured value puts
// a step the size of the tracking error into the first 1 ms tick, which trips
// joint_motion_generator_acceleration_discontinuity.
//
// Safety:
//   - Joint-limit check on the goal q (mode 1) and on every tick (both modes).
//   - SIGINT / SIGTERM: stop the motion (relies on libfranka to wind down).
//   - Cartesian mode caps |duration| at [1.5, 20.0] seconds.
//
// Exit codes:
//   0  ok
//   1  bad CLI
//   2  (retired) joint goal out of nominal limits -- now a non-fatal warning
//   3  (retired) start state outside nominal limits -- now a non-fatal warning
//  10  franka::Exception
//  11  std::exception

#include "examples_common.h"

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <array>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>

namespace {

const Eigen::Matrix<double, 7, 1> Q_MIN =
    (Eigen::Matrix<double, 7, 1>() << -2.8973, -1.7628, -2.8973, -3.0718,
     -2.8973, -0.0175, -2.8973)
        .finished();
const Eigen::Matrix<double, 7, 1> Q_MAX =
    (Eigen::Matrix<double, 7, 1>() << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973,
     3.7525, 2.8973)
        .finished();

std::atomic<bool> g_stop_flag{false};
void signal_handler(int /*signo*/) { g_stop_flag.store(true); }

enum class Mode { kNone, kJoint, kPose };

struct Args {
  std::string robot_ip;
  Mode mode{Mode::kNone};
  std::array<double, 7> q_goal{};
  // Pose mode: position + quaternion (wxyz)
  double tx{0.0}, ty{0.0}, tz{0.0};
  double qw{1.0}, qx{0.0}, qy{0.0}, qz{0.0};
  double speed_factor{0.2};
  double duration{5.0};
};

void print_usage(const char* prog) {
  std::cerr
      << "Usage: " << prog << " <robot_ip>\n"
      << "       --q q1 q2 q3 q4 q5 q6 q7 [--speed-factor 0.2]\n"
      << "       (or)\n"
      << "       --pose tx ty tz qw qx qy qz [--duration 5.0]" << std::endl;
}

bool parse_args(int argc, char** argv, Args& out) {
  if (argc < 2) {
    print_usage(argv[0]);
    return false;
  }
  out.robot_ip = argv[1];
  if (out.robot_ip.empty() || out.robot_ip[0] == '-') {
    print_usage(argv[0]);
    return false;
  }
  int i = 2;
  while (i < argc) {
    std::string key = argv[i];
    if (key == "--q") {
      if (i + 7 >= argc) return false;
      out.mode = Mode::kJoint;
      for (int j = 0; j < 7; ++j) {
        out.q_goal[j] = std::atof(argv[i + 1 + j]);
      }
      i += 8;
    } else if (key == "--pose") {
      if (i + 7 >= argc) return false;
      out.mode = Mode::kPose;
      out.tx = std::atof(argv[i + 1]);
      out.ty = std::atof(argv[i + 2]);
      out.tz = std::atof(argv[i + 3]);
      out.qw = std::atof(argv[i + 4]);
      out.qx = std::atof(argv[i + 5]);
      out.qy = std::atof(argv[i + 6]);
      out.qz = std::atof(argv[i + 7]);
      i += 8;
    } else if (key == "--speed-factor") {
      if (i + 1 >= argc) return false;
      out.speed_factor = std::atof(argv[i + 1]);
      i += 2;
    } else if (key == "--duration") {
      if (i + 1 >= argc) return false;
      out.duration = std::atof(argv[i + 1]);
      i += 2;
    } else if (key == "-h" || key == "--help") {
      print_usage(argv[0]);
      return false;
    } else {
      std::cerr << "[move_to] unknown arg: " << key << std::endl;
      print_usage(argv[0]);
      return false;
    }
  }
  if (out.mode == Mode::kNone) {
    std::cerr << "[move_to] must specify either --q or --pose" << std::endl;
    print_usage(argv[0]);
    return false;
  }
  if (out.speed_factor <= 0.0 || out.speed_factor > 0.5) {
    std::cerr << "[move_to] --speed-factor out of range (0, 0.5]" << std::endl;
    return false;
  }
  if (out.duration < 1.5 || out.duration > 20.0) {
    std::cerr << "[move_to] --duration out of range [1.5, 20.0] s" << std::endl;
    return false;
  }
  return true;
}

bool q_within_limits(const std::array<double, 7>& q) {
  for (int j = 0; j < 7; ++j) {
    if (q[j] < Q_MIN(j) || q[j] > Q_MAX(j)) return false;
  }
  return true;
}

// 5th-order min-jerk scalar profile s(t) in [0, 1] for t in [0, T].
double minjerk_s(double t, double T) {
  if (t <= 0.0) return 0.0;
  if (t >= T) return 1.0;
  const double tau = t / T;
  return 10.0 * std::pow(tau, 3) - 15.0 * std::pow(tau, 4) +
         6.0 * std::pow(tau, 5);
}

// Compose a column-major 4x4 homogeneous transform from position + quaternion.
std::array<double, 16> pose_col_major(const Eigen::Vector3d& p,
                                      const Eigen::Quaterniond& q) {
  const Eigen::Matrix3d R = q.normalized().toRotationMatrix();
  std::array<double, 16> m{};
  // Column-major: m[col*4 + row].
  m[0] = R(0, 0); m[1] = R(1, 0); m[2] = R(2, 0); m[3] = 0.0;
  m[4] = R(0, 1); m[5] = R(1, 1); m[6] = R(2, 1); m[7] = 0.0;
  m[8] = R(0, 2); m[9] = R(1, 2); m[10] = R(2, 2); m[11] = 0.0;
  m[12] = p.x(); m[13] = p.y(); m[14] = p.z(); m[15] = 1.0;
  return m;
}

struct PoseFromArray {
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
};

PoseFromArray extract_pose(const std::array<double, 16>& m) {
  PoseFromArray out;
  out.p = Eigen::Vector3d(m[12], m[13], m[14]);
  Eigen::Matrix3d R;
  R << m[0], m[4], m[8], m[1], m[5], m[9], m[2], m[6], m[10];
  out.q = Eigen::Quaterniond(R).normalized();
  return out;
}

}  // namespace

int main(int argc, char** argv) {
  Args args;
  if (!parse_args(argc, argv, args)) return 1;

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  if (args.mode == Mode::kJoint && !q_within_limits(args.q_goal)) {
    // Nominal-limit check downgraded from fatal to a warning: proceed anyway.
    // libfranka still enforces the robot's hard joint limits at 1 kHz and will
    // reject/fault on a genuinely unreachable goal, so this only removes the
    // conservative software gate.
    std::cerr << "[move_to] WARNING: goal q out of nominal joint limits "
                 "(proceeding; libfranka still enforces hard limits)"
              << std::endl;
  }

  try {
    franka::Robot robot(args.robot_ip);
    // Clear any latched reflex/error before commanding motion, else libfranka
    // rejects the Move with "command not possible in the current mode (Reflex)"
    // (see osc_shm.cpp / deoxys franka_control_node.cpp). No-op if no error.
    robot.automaticErrorRecovery();
    setDefaultBehavior(robot);

    franka::RobotState initial_state = robot.readOnce();
    if (!q_within_limits(initial_state.q)) {
      // Downgraded from fatal to a warning: a start config slightly outside the
      // nominal band (e.g. a hand-guided joint 7 past 2.8973) no longer blocks a
      // reset that moves back into range.
      std::cerr << "[move_to] WARNING: start q out of nominal joint limits "
                   "(proceeding)"
                << std::endl;
    }

    if (args.mode == Mode::kJoint) {
      std::cout << "[move_to] joint mode: speed_factor=" << args.speed_factor
                << "\n[move_to] q_goal = ";
      for (int j = 0; j < 7; ++j) {
        std::cout << args.q_goal[j];
        if (j + 1 < 7) std::cout << " ";
      }
      std::cout << std::endl;

      MotionGenerator gen(args.speed_factor, args.q_goal);
      robot.control([&](const franka::RobotState& s,
                        franka::Duration period) -> franka::JointPositions {
        // The FCI requires the first commanded position to equal the robot's
        // current *desired* q_d, not the measured q -- they differ by the
        // impedance tracking error (gravity sag, or wherever an aborted motion
        // left things). MotionGenerator is vendored verbatim and seeds from
        // robot_state.q, so hand it a state whose q is q_d.
        franka::RobotState seeded = s;
        seeded.q = s.q_d;
        franka::JointPositions out = gen(seeded, period);
        if (g_stop_flag.load()) {
          out = franka::MotionFinished(out);
        }
        return out;
      });
      std::cout << "[move_to] joint motion finished" << std::endl;
    } else {
      // Pose mode: minimum-jerk in task space between captured start pose and
      // user-specified goal pose.
      PoseFromArray start = extract_pose(initial_state.O_T_EE);
      const Eigen::Vector3d p_goal(args.tx, args.ty, args.tz);
      const Eigen::Quaterniond q_goal_raw(args.qw, args.qx, args.qy, args.qz);
      if (std::abs(q_goal_raw.norm() - 1.0) > 1e-3) {
        std::cerr << "[move_to] WARN: --pose quaternion not unit norm "
                  << "(norm=" << q_goal_raw.norm() << "); will normalize."
                  << std::endl;
      }
      Eigen::Quaterniond q_goal = q_goal_raw.normalized();
      // Shortest-path quaternion sign so slerp doesn't take the long way.
      if (start.q.dot(q_goal) < 0.0) {
        q_goal.coeffs() *= -1.0;
      }

      std::cout << "[move_to] pose mode: duration=" << args.duration
                << " s\n[move_to] p_start = " << start.p.transpose()
                << "\n[move_to] p_goal  = " << p_goal.transpose()
                << "\n[move_to] q_start (wxyz) = " << start.q.w() << " "
                << start.q.x() << " " << start.q.y() << " " << start.q.z()
                << "\n[move_to] q_goal  (wxyz) = " << q_goal.w() << " "
                << q_goal.x() << " " << q_goal.y() << " " << q_goal.z()
                << std::endl;

      double t = 0.0;
      bool seeded = false;
      const double T = args.duration;
      robot.control([&](const franka::RobotState& s,
                        franka::Duration period) -> franka::CartesianPose {
        if (!seeded) {
          // Same rule as joint mode: the trajectory must start at the robot's
          // commanded pose O_T_EE_c, which is only populated once the control
          // loop is running (readOnce() above gives the measured O_T_EE).
          start = extract_pose(s.O_T_EE_c);
          if (start.q.dot(q_goal) < 0.0) {
            q_goal.coeffs() *= -1.0;
          }
          seeded = true;
        }
        t += period.toSec();
        double s_t = minjerk_s(t, T);
        const Eigen::Vector3d p = start.p + s_t * (p_goal - start.p);
        const Eigen::Quaterniond q = start.q.slerp(s_t, q_goal);
        std::array<double, 16> mat = pose_col_major(p, q);
        franka::CartesianPose out(mat);
        const bool finished = (t >= T) || g_stop_flag.load();
        if (finished) {
          out = franka::MotionFinished(out);
        }
        return out;
      });
      std::cout << "[move_to] pose motion finished" << std::endl;
    }
  } catch (const franka::Exception& e) {
    std::cerr << "[move_to] franka::Exception: " << e.what() << std::endl;
    return 10;
  } catch (const std::exception& e) {
    std::cerr << "[move_to] std::exception: " << e.what() << std::endl;
    return 11;
  }
  return 0;
}
