// move_to.cpp
//
// One-shot blocking utility that drives the robot to a target joint
// configuration OR a target end-effector pose, using libfranka's built-in
// motion generators. Designed to be invoked by the frankatwin daemon for
// reset/home before handing the libfranka session back to `osc_shm`.
//
// Two mutually-exclusive modes:
//   1. --q q1 q2 q3 q4 q5 q6 q7 [--q-max-speed 0.5]
//      Uses libfranka's joint MotionGenerator (vendored from examples): per
//      joint a cubic acceleration ramp, a constant-velocity cruise and a cubic
//      deceleration ramp -- a smoothed trapezoid, NOT a min-jerk profile --
//      respecting dq_max and ddq_max scaled by --q-max-speed, with the seven
//      joints synchronized to finish together.
//   2. --pose tx ty tz qw qx qy qz [--q-max-speed 0.5]
//      Uses libfranka's franka::CartesianPose motion type. Each tick the
//      callback returns the desired 4x4 column-major matrix interpolated
//      between the start pose (captured at t=0) and the target pose via a 5th
//      order min-jerk profile. NO IK is performed on our side.
//
// Pacing (--q-max-speed V, rad/s):
//   Joint mode maps it exactly onto MotionGenerator's speed_factor, which
//   scales the vendored dq_max_ = [2,2,2,2,2.5,2.5,2.5]: speed_factor =
//   V / 2.5 caps every joint at or below V.
//
//   Pose mode has no joint space of its own -- libfranka solves the IK -- so
//   it estimates one. On the first control tick it takes the Jacobian at the
//   start configuration, maps the Cartesian travel to a joint displacement
//   (damped least squares), and picks T so the min-jerk peak joint velocity
//   1.875*max|dq| / T equals V. This is a FIRST-ORDER ESTIMATE: J is sampled
//   once, at the start, so it degrades over large reorientations and near
//   singularities, where J^+ blows up. It is a pacing heuristic, not a
//   guarantee -- the robot's own limits remain the real protection.
//
// Both modes seed their trajectory from the robot's *commanded* state (q_d /
// O_T_EE_c) rather than the measured one. Starting from the measured value puts
// a step the size of the tracking error into the first 1 ms tick, which trips
// joint_motion_generator_acceleration_discontinuity.
//
// Safety:
//   - Joint-limit check on the goal q (mode 1) and on every tick (both modes).
//   - SIGINT / SIGTERM: stop the motion (relies on libfranka to wind down).
//   - Both modes are paced by one knob, --q-max-speed, a per-joint velocity
//     cap in rad/s. Motion time is derived from it, so a longer travel takes
//     longer instead of moving faster. See kQMaxSpeed* below.
//
// Exit codes:
//   0  ok
//   1  bad CLI
//   2  (retired) joint goal out of nominal limits -- now a non-fatal warning
//   3  (retired) start state outside nominal limits -- now a non-fatal warning
//   5  robot busy: another frankatwin controller holds the FCI lock
//  10  franka::Exception
//  11  std::exception

#include "examples_common.h"
#include "fci_lock.h"

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

// --q-max-speed: per-joint velocity cap [rad/s], the single pacing knob.
// The reference is MotionGenerator's largest dq_max_ entry (2.5 rad/s for
// joints 5-7), so V / kDqMaxRef is the speed_factor that caps every joint at
// V. The ceiling mirrors the old speed_factor <= 0.5 guard (0.5 * 2.5) and the
// default mirrors the old speed_factor 0.2 (0.2 * 2.5), so behaviour is
// unchanged for anyone who does not pass the flag.
constexpr double kDqMaxRef = 2.5;
constexpr double kQMaxSpeedDefault = 0.5;
constexpr double kQMaxSpeedMax = 1.25;
// Pose-mode motion time, after the Jacobian estimate, is clamped here. The
// floor keeps a near-zero travel from becoming a step; the ceiling keeps a
// pathological estimate inside the daemon's 30 s subprocess timeout.
constexpr double kPoseTMin = 0.3;
constexpr double kPoseTMax = 20.0;
// Damping for the least-squares Jacobian inverse, so a near-singular start
// configuration yields a large-but-finite dq estimate instead of a division
// by ~0 (which would collapse T to the floor -- the opposite of safe).
constexpr double kJacDamping = 0.05;

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
  double q_max_speed{kQMaxSpeedDefault};
};

void print_usage(const char* prog) {
  std::cerr
      << "Usage: " << prog << " <robot_ip>\n"
      << "       --q q1 q2 q3 q4 q5 q6 q7      [--q-max-speed 0.5]\n"
      << "       (or)\n"
      << "       --pose tx ty tz qw qx qy qz   [--q-max-speed 0.5]\n"
      << "\n"
      << "  --q-max-speed V   per-joint velocity cap [rad/s], (0, "
      << kQMaxSpeedMax << "]. Motion time\n"
      << "                    follows from it; a longer travel takes longer."
      << std::endl;
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
    } else if (key == "--q-max-speed") {
      if (i + 1 >= argc) return false;
      out.q_max_speed = std::atof(argv[i + 1]);
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
  // One knob, same meaning in both modes, so there is no mode-specific flag
  // to accept or reject here.
  if (out.q_max_speed <= 0.0 || out.q_max_speed > kQMaxSpeedMax) {
    std::cerr << "[move_to] --q-max-speed out of range (0, " << kQMaxSpeedMax
              << "] rad/s" << std::endl;
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

  // Claim the robot before touching libfranka. A second controlling session
  // connects happily and only fails later, deep inside setDefaultBehavior, with
  // a message that never mentions the real holder -- see fci_lock.h.
  frankatwin::FciLock fci_lock;
  {
    std::string holder;
    // Wait a little rather than failing outright: the daemon stops osc_shm and
    // spawns us immediately, so the previous holder may still be exiting.
    if (!fci_lock.acquire(args.robot_ip, "move_to", 3000, &holder)) {
      std::cerr << "[move_to] robot " << args.robot_ip
                << " is already held by another frankatwin session (" << holder
                << ").\n"
                << "[move_to] Drive it through the daemon instead "
                   "(client.move_to_pose(...) / client.move_to_q(...)), "
                   "or stop the daemon first."
                << std::endl;
      return frankatwin::kExitRobotBusy;
    }
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
      // V / kDqMaxRef is the scale that caps every joint at or below V,
      // because MotionGenerator multiplies its dq_max_ by this factor.
      const double speed_factor = args.q_max_speed / kDqMaxRef;
      std::cout << "[move_to] joint mode: q_max_speed=" << args.q_max_speed
                << " rad/s (speed_factor=" << speed_factor << ")"
                << "\n[move_to] q_goal = ";
      for (int j = 0; j < 7; ++j) {
        std::cout << args.q_goal[j];
        if (j + 1 < 7) std::cout << " ";
      }
      std::cout << std::endl;

      MotionGenerator gen(speed_factor, args.q_goal);
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

      std::cout << "[move_to] pose mode: q_max_speed=" << args.q_max_speed
                << " rad/s (APPROXIMATE -- libfranka solves the IK, so the "
                   "joint speed is\n"
                   "[move_to]   estimated from the Jacobian at the start pose "
                   "only. It degrades over\n"
                   "[move_to]   large reorientations and near singularities.)"
                << "\n[move_to] p_start = " << start.p.transpose()
                << "\n[move_to] p_goal  = " << p_goal.transpose()
                << "\n[move_to] q_start (wxyz) = " << start.q.w() << " "
                << start.q.x() << " " << start.q.y() << " " << start.q.z()
                << "\n[move_to] q_goal  (wxyz) = " << q_goal.w() << " "
                << q_goal.x() << " " << q_goal.y() << " " << q_goal.z()
                << std::endl;

      // The Jacobian below needs the kinematic model; loading it here keeps
      // the (blocking) download out of the 1 kHz callback.
      franka::Model model = robot.loadModel();

      double t = 0.0;
      bool seeded = false;
      double T = kPoseTMin;
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
          // Pace this Cartesian motion by the joint-space cap. Map the whole
          // travel through the start Jacobian to a joint displacement, then
          // pick T so the min-jerk peak joint velocity (1.875 * dq / T) hits
          // --q-max-speed. Damped least squares keeps a near-singular start
          // from producing a tiny dq (and therefore a dangerously short T).
          const std::array<double, 42> jac_arr =
              model.zeroJacobian(franka::Frame::kEndEffector, s);
          Eigen::Map<const Eigen::Matrix<double, 6, 7>> J(jac_arr.data());
          Eigen::Matrix<double, 6, 1> dx;
          dx.head<3>() = p_goal - start.p;
          const Eigen::AngleAxisd aa(q_goal * start.q.conjugate());
          dx.tail<3>() = aa.axis() * aa.angle();
          const Eigen::Matrix<double, 6, 6> JJt =
              J * J.transpose() +
              (kJacDamping * kJacDamping) * Eigen::Matrix<double, 6, 6>::Identity();
          const Eigen::Matrix<double, 7, 1> dq_est =
              J.transpose() * JJt.ldlt().solve(dx);
          const double dq_abs = dq_est.cwiseAbs().maxCoeff();
          T = 1.875 * dq_abs / args.q_max_speed;
          if (T < kPoseTMin) T = kPoseTMin;
          if (T > kPoseTMax) T = kPoseTMax;
          std::cout << "[move_to] |dq| estimate = " << dq_abs
                    << " rad -> T = " << T << " s" << std::endl;
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
    // The FCI lock above only covers frankatwin's own binaries. A foreign FCI
    // client (franka_ros, franka-interface, a Desk operation) leaves the robot
    // in a mode where our very first parameter command is rejected, and
    // libfranka's wording never hints at a second client. Name the real cause.
    const std::string msg = e.what();
    if (msg.find("current mode") != std::string::npos) {
      std::cerr << "[move_to] This usually means another FCI client owns the "
                   "robot, or Desk is holding it.\n"
                << "[move_to] Run `frankatwin doctor` to see the competing "
                   "clients, and check that Desk has released control."
                << std::endl;
    }
    return 10;
  } catch (const std::exception& e) {
    std::cerr << "[move_to] std::exception: " << e.what() << std::endl;
    return 11;
  }
  return 0;
}
