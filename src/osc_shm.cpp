// osc_shm.cpp
//
// Long-running 1 kHz Jacobian-transpose Cartesian 6D pose impedance controller
// that reads its setpoint and gains from POSIX shared memory.
//
// Control law (identical to step5b_cart_pose.cpp):
//   tau_cmd = J^T * F_task + c(q, dq)
//   F_task  = [ Kp_pos * (x_des - x) - Kd_pos * v ;
//               Kp_ori * e_o          - Kd_ori * w ]
//   e_o     = 2 * vec(q_des * q^{-1}) with shortest-path sign
//
// Difference vs step5b:
//   - x_des / q_des / Kp_pos / Kp_ori / Kd_pos / Kd_ori / enabled come from
//     PandaShmCommand each tick (lock-free seqlock read).
//   - q / dq / ee_pos / ee_quat / tau / timestamp are published to the state
//     ring buffer each tick.
//   - When `enabled == 0`, the controller outputs zero command torque (the
//     robot is held only by libfranka's gravity + friction compensation).
//
// Safety semantics preserved:
//   - Per-tick joint-limit check (Q_MIN / Q_MAX).
//   - Per-tick |e_pos|_inf > error_delta_pos (if set) and ||e_o|| >
//     error_delta_rot (if set) raise an abort.
//   - SIGINT / SIGTERM: clean stop with zero torque.
//
// CLI:
//   ./osc_shm <robot_ip>
//       [--shm-name NAME]       POSIX shm name (default "/panda_osc")
//       [--init-shm]            create + zero the shm before opening
//                               (when daemon owns shm, omit this flag)
//       [--no-coriolis]         disable explicit Coriolis term
//       [--print-every N]       diagnostic print every N ticks (0=off)
//       [--duration sec]        0 = until SIGINT/SIGTERM
//
// All control parameters (Kp/Kd, target pose) come from shm at runtime.
// Initial pose target is captured at startup from forward kinematics so the
// robot holds in place until a real client takes over.

#include "shm_layout.h"

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>

namespace {

const Eigen::Matrix<double, 7, 1> TAU_LIMIT =
    (Eigen::Matrix<double, 7, 1>() << 87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0)
        .finished();
const Eigen::Matrix<double, 7, 1> Q_MIN =
    (Eigen::Matrix<double, 7, 1>() << -2.8973, -1.7628, -2.8973, -3.0718,
     -2.8973, -0.0175, -2.8973)
        .finished();
const Eigen::Matrix<double, 7, 1> Q_MAX =
    (Eigen::Matrix<double, 7, 1>() << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973,
     3.7525, 2.8973)
        .finished();

constexpr double DEFAULT_KP_POS = 200.0;
constexpr double DEFAULT_KP_ORI = 20.0;

std::atomic<bool> g_stop_flag{false};
void signal_handler(int /*signo*/) { g_stop_flag.store(true); }

struct Args {
  std::string robot_ip;
  std::string shm_name{PANDA_SHM_DEFAULT_NAME};
  bool init_shm{false};
  bool no_coriolis{false};
  int print_every{0};
  double duration{0.0};
};

void print_usage(const char* prog) {
  std::cerr << "Usage: " << prog << " <robot_ip>"
            << " [--shm-name NAME] [--init-shm] [--no-coriolis]"
            << " [--print-every N] [--duration sec]" << std::endl;
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
    if (key == "--shm-name") {
      if (i + 1 >= argc) return false;
      out.shm_name = argv[i + 1];
      i += 2;
    } else if (key == "--init-shm") {
      out.init_shm = true;
      i += 1;
    } else if (key == "--no-coriolis") {
      out.no_coriolis = true;
      i += 1;
    } else if (key == "--print-every") {
      if (i + 1 >= argc) return false;
      out.print_every = std::atoi(argv[i + 1]);
      i += 2;
    } else if (key == "--duration") {
      if (i + 1 >= argc) return false;
      out.duration = std::atof(argv[i + 1]);
      i += 2;
    } else if (key == "-h" || key == "--help") {
      print_usage(argv[0]);
      return false;
    } else {
      std::cerr << "[osc_shm] unknown arg: " << key << std::endl;
      print_usage(argv[0]);
      return false;
    }
  }
  return true;
}

bool try_set_realtime_priority(int priority = 80) {
  sched_param sp{};
  sp.sched_priority = priority;
  int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
  if (rc != 0) {
    std::cerr << "[osc_shm] WARN: SCHED_FIFO failed (rc=" << rc
              << ", errno=" << std::strerror(errno) << "). Continuing non-RT."
              << std::endl;
    return false;
  }
  return true;
}

bool q_within_limits(const Eigen::Matrix<double, 7, 1>& q) {
  for (int j = 0; j < 7; ++j) {
    if (q(j) < Q_MIN(j) || q(j) > Q_MAX(j)) return false;
  }
  return true;
}

struct PoseData {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond quaternion{Eigen::Quaterniond::Identity()};
};

PoseData extract_pose_data(const std::array<double, 16>& m) {
  PoseData p;
  p.position = Eigen::Vector3d(m[12], m[13], m[14]);
  Eigen::Matrix3d R;
  R << m[0], m[4], m[8], m[1], m[5], m[9], m[2], m[6], m[10];
  p.quaternion = Eigen::Quaterniond(R);
  p.quaternion.normalize();
  return p;
}

Eigen::Vector3d shortest_quat_error_vec(const Eigen::Quaterniond& q_des,
                                        Eigen::Quaterniond q_cur) {
  q_cur.normalize();
  Eigen::Quaterniond q_err = q_des * q_cur.conjugate();
  if (q_err.w() < 0.0) {
    q_err.coeffs() *= -1.0;
  }
  return 2.0 * q_err.vec();
}

double monotonic_seconds() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<double>(ts.tv_sec) +
         static_cast<double>(ts.tv_nsec) * 1e-9;
}

// Open (or create) the POSIX shm segment and return a memory-mapped pointer.
// On failure, prints an error and returns nullptr.
PandaShm* open_shm(const std::string& name, bool create_init, int* out_fd) {
  *out_fd = -1;
  int oflag = create_init ? (O_CREAT | O_RDWR) : O_RDWR;
  int fd = shm_open(name.c_str(), oflag, 0666);
  if (fd < 0) {
    std::cerr << "[osc_shm] shm_open('" << name
              << "') failed: " << std::strerror(errno) << std::endl;
    return nullptr;
  }
  if (create_init) {
    if (ftruncate(fd, static_cast<off_t>(sizeof(PandaShm))) != 0) {
      std::cerr << "[osc_shm] ftruncate failed: " << std::strerror(errno)
                << std::endl;
      ::close(fd);
      return nullptr;
    }
  } else {
    struct stat st{};
    if (fstat(fd, &st) != 0 || static_cast<size_t>(st.st_size) != sizeof(PandaShm)) {
      std::cerr << "[osc_shm] shm '" << name << "' has unexpected size "
                << (st.st_size) << " (expected " << sizeof(PandaShm)
                << ", did you forget --init-shm or run an older daemon?)"
                << std::endl;
      ::close(fd);
      return nullptr;
    }
  }
  void* mapped = mmap(nullptr, sizeof(PandaShm), PROT_READ | PROT_WRITE,
                      MAP_SHARED, fd, 0);
  if (mapped == MAP_FAILED) {
    std::cerr << "[osc_shm] mmap failed: " << std::strerror(errno) << std::endl;
    ::close(fd);
    return nullptr;
  }
  *out_fd = fd;
  PandaShm* shm = reinterpret_cast<PandaShm*>(mapped);
  if (create_init) {
    std::memset(shm, 0, sizeof(PandaShm));
    shm->header.magic = PANDA_SHM_MAGIC;
    shm->header.version = PANDA_SHM_VERSION;
    shm->header.state_frames = PANDA_SHM_STATE_FRAMES;
  } else {
    if (shm->header.magic != PANDA_SHM_MAGIC ||
        shm->header.version != PANDA_SHM_VERSION) {
      std::cerr << "[osc_shm] shm header mismatch: magic=0x" << std::hex
                << shm->header.magic << " version=" << std::dec
                << shm->header.version << std::endl;
      munmap(mapped, sizeof(PandaShm));
      ::close(fd);
      return nullptr;
    }
  }
  return shm;
}

}  // namespace

int main(int argc, char** argv) {
  Args args;
  if (!parse_args(argc, argv, args)) return 1;

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  int shm_fd = -1;
  PandaShm* shm = open_shm(args.shm_name, args.init_shm, &shm_fd);
  if (shm == nullptr) return 2;

  // Stamp our pid into the header so observers know the controller is alive.
  __atomic_store_n(&shm->header.controller_pid,
                   static_cast<uint64_t>(getpid()), __ATOMIC_RELEASE);

  std::cout << "[osc_shm] robot_ip = " << args.robot_ip << "\n"
            << "[osc_shm] shm_name = " << args.shm_name
            << (args.init_shm ? " (created)" : " (opened)") << "\n"
            << "[osc_shm] pid      = " << getpid() << std::endl;

  const bool rt_ok = try_set_realtime_priority();
  std::cout << "[osc_shm] RT       = " << (rt_ok ? "SCHED_FIFO" : "non-RT")
            << std::endl;

  const double t_start_mono = monotonic_seconds();

  try {
    franka::Robot robot(args.robot_ip);
    robot.setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}});

    franka::Model model = robot.loadModel();
    franka::RobotState initial_state = robot.readOnce();

    Eigen::Map<const Eigen::Matrix<double, 7, 1>> q_init(initial_state.q.data());
    if (!q_within_limits(q_init)) {
      std::cerr << "[osc_shm] q_init exceeds nominal Panda joint limits"
                << std::endl;
      return 4;
    }

    const PoseData anchor =
        extract_pose_data(model.pose(franka::Frame::kEndEffector, initial_state));

    // Seed the command with the anchor pose + default gains so that the
    // controller holds in place until a client overrides the setpoint.
    {
      PandaShmCommand* cmd = &shm->command;
      uint64_t s = panda_shm::cmd_write_begin(cmd);
      cmd->target_pos[0] = anchor.position.x();
      cmd->target_pos[1] = anchor.position.y();
      cmd->target_pos[2] = anchor.position.z();
      cmd->target_quat[0] = anchor.quaternion.w();
      cmd->target_quat[1] = anchor.quaternion.x();
      cmd->target_quat[2] = anchor.quaternion.y();
      cmd->target_quat[3] = anchor.quaternion.z();
      cmd->kp_pos = DEFAULT_KP_POS;
      cmd->kp_ori = DEFAULT_KP_ORI;
      cmd->kd_pos = 0.0;  // 0 -> auto 2*sqrt(kp)
      cmd->kd_ori = 0.0;
      cmd->error_delta_pos = 0.05;
      cmd->error_delta_rot = 0.30;
      cmd->enabled = 1u;
      panda_shm::cmd_write_end(cmd, s);
    }

    std::cout << "[osc_shm] q_init    = " << q_init.transpose() << "\n"
              << "[osc_shm] x_anchor  = " << anchor.position.transpose() << "\n"
              << "[osc_shm] quat (wxyz) = " << anchor.quaternion.w() << " "
              << anchor.quaternion.x() << " " << anchor.quaternion.y() << " "
              << anchor.quaternion.z() << "\n"
              << "[osc_shm] starting 1 kHz loop. SIGINT to stop." << std::endl;

    size_t tick = 0;
    double elapsed = 0.0;
    int abort_code = 0;
    double abort_value = 0.0;
    int abort_joint = -1;

    auto callback = [&](const franka::RobotState& s,
                        franka::Duration period) -> franka::Torques {
      elapsed += period.toSec();

      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(s.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(s.dq.data());

      const PoseData pose =
          extract_pose_data(model.pose(franka::Frame::kEndEffector, s));
      const Eigen::Vector3d x = pose.position;
      const Eigen::Quaterniond q_cur = pose.quaternion;

      const std::array<double, 42> jac_arr =
          model.zeroJacobian(franka::Frame::kEndEffector, s);
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jac6x7(jac_arr.data());
      const Eigen::Vector3d v = jac6x7.topRows<3>() * dq;
      const Eigen::Vector3d w = jac6x7.bottomRows<3>() * dq;

      // Snapshot the command (lock-free seqlock).
      PandaShmCommand cmd_snap;
      panda_shm::cmd_read(&shm->command, &cmd_snap);

      const Eigen::Vector3d x_des(cmd_snap.target_pos[0],
                                  cmd_snap.target_pos[1],
                                  cmd_snap.target_pos[2]);
      Eigen::Quaterniond q_des(cmd_snap.target_quat[0],
                               cmd_snap.target_quat[1],
                               cmd_snap.target_quat[2],
                               cmd_snap.target_quat[3]);
      q_des.normalize();

      const double kp_pos = cmd_snap.kp_pos;
      const double kp_ori = cmd_snap.kp_ori;
      const double kd_pos =
          cmd_snap.kd_pos > 0.0 ? cmd_snap.kd_pos : 2.0 * std::sqrt(kp_pos);
      const double kd_ori =
          cmd_snap.kd_ori > 0.0 ? cmd_snap.kd_ori : 2.0 * std::sqrt(kp_ori);
      const double err_dp = cmd_snap.error_delta_pos;
      const double err_dr = cmd_snap.error_delta_rot;

      Eigen::Vector3d e_pos = x_des - x;
      Eigen::Vector3d e_ori = shortest_quat_error_vec(q_des, q_cur);
      if (err_dp > 0.0) {
        e_pos = e_pos.cwiseMax(-err_dp).cwiseMin(err_dp);
      }
      if (err_dr > 0.0) {
        e_ori = e_ori.cwiseMax(-err_dr).cwiseMin(err_dr);
      }

      Eigen::Matrix<double, 6, 1> f_task;
      f_task.head<3>() = kp_pos * e_pos - kd_pos * v;
      f_task.tail<3>() = kp_ori * e_ori - kd_ori * w;

      Eigen::Matrix<double, 7, 1> c_vec = Eigen::Matrix<double, 7, 1>::Zero();
      if (!args.no_coriolis) {
        const std::array<double, 7> c_arr = model.coriolis(s);
        c_vec = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(c_arr.data());
      }

      Eigen::Matrix<double, 7, 1> tau_pd =
          jac6x7.transpose() * f_task + c_vec;
      if (cmd_snap.enabled == 0u) {
        // Hold mode: zero command torque (libfranka still adds gravity + friction).
        tau_pd.setZero();
      }
      const Eigen::Matrix<double, 7, 1> tau_cmd =
          tau_pd.cwiseMax(-TAU_LIMIT).cwiseMin(TAU_LIMIT);

      // Publish the state frame to the ring buffer.
      PandaShmStateFrame frame{};
      frame.timestamp_s = monotonic_seconds() - t_start_mono;
      for (int j = 0; j < 7; ++j) {
        frame.q[j] = s.q[j];
        frame.dq[j] = s.dq[j];
        frame.tau[j] = tau_cmd(j);
      }
      frame.ee_pos[0] = x.x();
      frame.ee_pos[1] = x.y();
      frame.ee_pos[2] = x.z();
      frame.ee_quat[0] = q_cur.w();
      frame.ee_quat[1] = q_cur.x();
      frame.ee_quat[2] = q_cur.y();
      frame.ee_quat[3] = q_cur.z();
      panda_shm::state_publish(&shm->header, shm->states, frame,
                               PANDA_SHM_STATE_FRAMES);

      // Periodic diagnostic print (off by default).
      if (args.print_every > 0 &&
          tick % static_cast<size_t>(args.print_every) == 0) {
        std::cout << "[osc_shm] t=" << std::fixed << std::setprecision(3)
                  << elapsed << " enabled=" << cmd_snap.enabled
                  << " kp_pos=" << kp_pos << " |e_pos|_inf="
                  << std::setprecision(6) << e_pos.cwiseAbs().maxCoeff()
                  << " ||e_o||=" << e_ori.norm() << std::endl;
      }

      // Safety checks (preserve step5b semantics).
      if (abort_code == 0 && !q_within_limits(q)) {
        abort_code = 2;
        for (int j = 0; j < 7; ++j) {
          if (q(j) < Q_MIN(j) || q(j) > Q_MAX(j)) {
            abort_joint = j;
            abort_value = q(j);
            break;
          }
        }
      }
      if (abort_code == 0 && err_dp > 0.0 &&
          (x_des - x).cwiseAbs().maxCoeff() > err_dp + 1e-9) {
        // Note: we already clipped the error term; this only fires if the
        // *unclipped* tracking error exceeds the threshold which means the
        // robot is being commanded far outside its allowed envelope.
        abort_code = 1;
        abort_value = (x_des - x).cwiseAbs().maxCoeff();
      }
      if (abort_code == 0 && err_dr > 0.0 &&
          shortest_quat_error_vec(q_des, q_cur).norm() > err_dr + 1e-9) {
        abort_code = 3;
        abort_value = shortest_quat_error_vec(q_des, q_cur).norm();
      }

      const bool time_up = (args.duration > 0.0 && elapsed >= args.duration);
      ++tick;

      if (g_stop_flag.load() || time_up || abort_code != 0) {
        std::array<double, 7> zero{};
        return franka::MotionFinished(franka::Torques(zero));
      }

      std::array<double, 7> tau_out{};
      Eigen::Map<Eigen::Matrix<double, 7, 1>>(tau_out.data()) = tau_cmd;
      return franka::Torques(tau_out);
    };

    robot.control(callback);

    std::cout << "\n[osc_shm] --- summary ---\n"
              << "ticks    : " << tick << "\n"
              << "elapsed  : " << elapsed << " s\n";
    if (abort_code == 1) {
      std::cout << "abort    : position tracking exceeded "
                << "(value=" << abort_value << ")" << std::endl;
    } else if (abort_code == 2) {
      std::cout << "abort    : joint limit (joint=" << (abort_joint + 1)
                << ", q=" << abort_value << ")" << std::endl;
    } else if (abort_code == 3) {
      std::cout << "abort    : orientation tracking exceeded "
                << "(value=" << abort_value << ")" << std::endl;
    } else {
      std::cout << "abort    : none" << std::endl;
    }
  } catch (const franka::Exception& e) {
    std::cerr << "[osc_shm] franka::Exception: " << e.what() << std::endl;
    __atomic_store_n(&shm->header.controller_pid, 0ull, __ATOMIC_RELEASE);
    munmap(shm, sizeof(PandaShm));
    ::close(shm_fd);
    return 10;
  } catch (const std::exception& e) {
    std::cerr << "[osc_shm] std::exception: " << e.what() << std::endl;
    __atomic_store_n(&shm->header.controller_pid, 0ull, __ATOMIC_RELEASE);
    munmap(shm, sizeof(PandaShm));
    ::close(shm_fd);
    return 11;
  }

  __atomic_store_n(&shm->header.controller_pid, 0ull, __ATOMIC_RELEASE);
  munmap(shm, sizeof(PandaShm));
  ::close(shm_fd);
  // NOTE: we do NOT shm_unlink here. The owner of the segment (typically the
  // daemon or whoever launched us with --init-shm) is responsible for cleanup.
  return 0;
}
