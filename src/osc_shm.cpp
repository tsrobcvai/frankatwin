// osc_shm.cpp
//
// Long-running 1 kHz Jacobian-transpose Cartesian 6D pose impedance controller
// that reads its setpoint and gains from POSIX shared memory.
//
// Control law:
//   tau_cmd = J^T * F_task + c(q, dq)
//   F_task  = [ Kp_pos * (x_des - x) - Kd_pos * v ;
//               Kp_ori * e_o          - Kd_ori * w ]
//   e_o     = 2 * vec(q_des * q^{-1}) with shortest-path sign
//
// Design:
//   - x_des / q_des / Kp_pos / Kp_ori / Kd_pos / Kd_ori / enabled come from
//     ShmCommand each tick (lock-free seqlock read).
//   - q / dq / ee_pos / ee_quat / tau / timestamp are published to the state
//     ring buffer each tick.
//   - When `enabled == 0`, the controller outputs zero command torque (the
//     robot is held only by libfranka's gravity + friction compensation).
//
// Safety semantics preserved:
//   - Per-tick joint-limit check (Q_MIN / Q_MAX) is WARN-ONLY: an out-of-nominal
//     joint logs once but no longer aborts (libfranka hard limits still apply).
//   - Per-tick |e_pos|_inf > error_delta_pos (if set) raises an abort. The
//     orientation channel is pure impedance: no clip, no tracking abort.
//   - SIGINT / SIGTERM: clean stop with zero torque.
//
// CLI:
//   ./osc_shm <robot_ip>
//       [--shm-name NAME]       POSIX shm name (default "/frankatwin_osc")
//       [--init-shm]            create + zero the shm before opening
//                               (when daemon owns shm, omit this flag)
//       [--no-coriolis]         disable explicit Coriolis term
//       [--print-every N]       diagnostic print every N ticks (0=off)
//       [--duration sec]        0 = until SIGINT/SIGTERM
//
// All control parameters (Kp/Kd, target pose) come from shm at runtime.
// Initial pose target is captured at startup from forward kinematics so the
// robot holds in place until a real client takes over.

#include "fci_lock.h"
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
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
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

// Commanded-torque slew-rate limit [Nm/s], per joint.  libfranka aborts with
// the "controller_torque_discontinuity" reflex when |dtau_J_d/dt| exceeds its
// internal kMaxTorqueRate (1000 Nm/s).  We default to 800 (20% margin) so the
// impedance law's torque steps (at setpoint/gain jumps) ramp over a few ms
// instead of stepping.  Set to <= 0 to disable the limiter.
constexpr double DEFAULT_MAX_TORQUE_RATE = 800.0;

// Controlled-stop settle thresholds.  When a stop is requested (SIGINT/SIGTERM,
// duration elapsed, or a safety abort) we do NOT hard-return MotionFinished with
// a zero-torque STEP -- that step trips libfranka's controller_torque_discontinuity
// reflex and, if the arm is still moving, also throws "robot is still moving".
// Instead the slew limiter ramps the command torque to zero and we only declare
// the motion finished once BOTH the command torque and the joint velocity have
// settled below these thresholds (or STOP_MAX_TICKS elapses as a hard cap).
constexpr double TAU_SETTLE_EPS = 0.05;   // Nm,   |tau_cmd|_inf considered ~0
constexpr double DQ_SETTLE_EPS = 0.05;    // rad/s, |dq|_inf considered at rest
constexpr int STOP_MAX_TICKS = 1000;      // ~1 s hard cap on the ramp-down

std::atomic<bool> g_stop_flag{false};
void signal_handler(int /*signo*/) { g_stop_flag.store(true); }

struct Args {
  std::string robot_ip;
  std::string shm_name{FRANKATWIN_SHM_DEFAULT_NAME};
  bool init_shm{false};
  bool no_coriolis{false};
  int print_every{0};
  double duration{0.0};
  double max_torque_rate{DEFAULT_MAX_TORQUE_RATE};
  // End-effector payload (mounted camera, etc.). mass<=0 -> don't call setLoad.
  double load_mass{0.0};
  std::array<double, 3> load_com{{0.0, 0.0, 0.0}};       // flange->load COM [m]
  std::array<double, 9> load_inertia{{0, 0, 0, 0, 0, 0, 0, 0, 0}};  // about COM [kg m^2]
  // libfranka collision-reflex thresholds. Each scalar fills ALL entries of
  // setCollisionBehavior's torque (7) / Cartesian (6) lower & upper, nominal &
  // acceleration arrays. The task-impedance controller's own push is bounded by
  // kp*error_delta (~25 N / ~9 Nm at kp_pos=500/kp_ori=30), so the legacy
  // default 20 trips "cartesian_reflex" during insertion contact. robot.yaml
  // raises these (see its collision: block); the 20 default here only applies
  // to a bare `osc_shm <ip>` call with no flag.
  double collision_torque{20.0};      // Nm, per joint
  double collision_cartesian{20.0};   // N / Nm, per Cartesian axis
};

void print_usage(const char* prog) {
  std::cerr << "Usage: " << prog << " <robot_ip>"
            << " [--shm-name NAME] [--init-shm] [--no-coriolis]"
            << " [--print-every N] [--duration sec]"
            << " [--max-torque-rate Nm_per_s]"
            << " [--load-mass kg] [--load-com x y z]"
            << " [--load-inertia i0 .. i8]"
            << " [--collision-torque Nm] [--collision-cartesian N]" << std::endl;
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
    } else if (key == "--max-torque-rate") {
      if (i + 1 >= argc) return false;
      out.max_torque_rate = std::atof(argv[i + 1]);
      i += 2;
    } else if (key == "--load-mass") {
      if (i + 1 >= argc) return false;
      out.load_mass = std::atof(argv[i + 1]);
      i += 2;
    } else if (key == "--load-com") {
      if (i + 3 >= argc) return false;
      out.load_com = {std::atof(argv[i + 1]), std::atof(argv[i + 2]),
                      std::atof(argv[i + 3])};
      i += 4;
    } else if (key == "--load-inertia") {
      if (i + 9 >= argc) return false;
      for (int k = 0; k < 9; ++k) out.load_inertia[k] = std::atof(argv[i + 1 + k]);
      i += 10;
    } else if (key == "--collision-torque") {
      if (i + 1 >= argc) return false;
      out.collision_torque = std::atof(argv[i + 1]);
      i += 2;
    } else if (key == "--collision-cartesian") {
      if (i + 1 >= argc) return false;
      out.collision_cartesian = std::atof(argv[i + 1]);
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
ShmSegment* open_shm(const std::string& name, bool create_init, int* out_fd) {
  *out_fd = -1;
  int oflag = create_init ? (O_CREAT | O_RDWR) : O_RDWR;
  int fd = shm_open(name.c_str(), oflag, 0666);
  if (fd < 0) {
    std::cerr << "[osc_shm] shm_open('" << name
              << "') failed: " << std::strerror(errno) << std::endl;
    return nullptr;
  }
  if (create_init) {
    if (ftruncate(fd, static_cast<off_t>(sizeof(ShmSegment))) != 0) {
      std::cerr << "[osc_shm] ftruncate failed: " << std::strerror(errno)
                << std::endl;
      ::close(fd);
      return nullptr;
    }
  } else {
    struct stat st{};
    if (fstat(fd, &st) != 0 || static_cast<size_t>(st.st_size) != sizeof(ShmSegment)) {
      std::cerr << "[osc_shm] shm '" << name << "' has unexpected size "
                << (st.st_size) << " (expected " << sizeof(ShmSegment)
                << ", did you forget --init-shm or run an older daemon?)"
                << std::endl;
      ::close(fd);
      return nullptr;
    }
  }
  void* mapped = mmap(nullptr, sizeof(ShmSegment), PROT_READ | PROT_WRITE,
                      MAP_SHARED, fd, 0);
  if (mapped == MAP_FAILED) {
    std::cerr << "[osc_shm] mmap failed: " << std::strerror(errno) << std::endl;
    ::close(fd);
    return nullptr;
  }
  *out_fd = fd;
  ShmSegment* shm = reinterpret_cast<ShmSegment*>(mapped);
  if (create_init) {
    std::memset(shm, 0, sizeof(ShmSegment));
    shm->header.magic = FRANKATWIN_SHM_MAGIC;
    shm->header.version = FRANKATWIN_SHM_VERSION;
    shm->header.state_frames = FRANKATWIN_SHM_STATE_FRAMES;
  } else {
    if (shm->header.magic != FRANKATWIN_SHM_MAGIC ||
        shm->header.version != FRANKATWIN_SHM_VERSION) {
      std::cerr << "[osc_shm] shm header mismatch: magic=0x" << std::hex
                << shm->header.magic << " version=" << std::dec
                << shm->header.version << std::endl;
      munmap(mapped, sizeof(ShmSegment));
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

  // Claim the robot before allocating anything. Two controllers on one robot
  // is never valid, and the FCI reports the clash only much later as a mode
  // error that names no one -- see fci_lock.h.
  frankatwin::FciLock fci_lock;
  {
    std::string holder;
    // Brief wait: the daemon relaunches us right after a move_to, which may
    // still be releasing the lock as it exits.
    if (!fci_lock.acquire(args.robot_ip, "osc_shm", 3000, &holder)) {
      std::cerr << "[osc_shm] robot " << args.robot_ip
                << " is already held by another frankatwin session (" << holder
                << ").\n"
                << "[osc_shm] Stop the running daemon/controller before "
                   "starting a second one."
                << std::endl;
      return frankatwin::kExitRobotBusy;
    }
  }

  int shm_fd = -1;
  ShmSegment* shm = open_shm(args.shm_name, args.init_shm, &shm_fd);
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
  std::cout << "[osc_shm] tau_rate = "
            << (args.max_torque_rate > 0.0
                    ? std::to_string(args.max_torque_rate) + " Nm/s (slew limit)"
                    : std::string("OFF"))
            << std::endl;

  const double t_start_mono = monotonic_seconds();

  try {
    franka::Robot robot(args.robot_ip);

    // Clear any latched reflex/error from a prior run (e.g. a collision reflex)
    // so Move commands aren't rejected with "command not possible in the current
    // mode (Reflex)". Without this, one reflex bricks every subsequent osc_shm /
    // move_to until the daemon is restarted. Mirrors deoxys
    // franka_control_node.cpp; a no-op when the robot has no active error.
    robot.automaticErrorRecovery();

    // Register an end-effector payload (e.g. a mounted camera) so libfranka's
    // gravity/inertia compensation accounts for it.  Without this, the extra
    // weight is uncompensated and the impedance controller sags (notably in z).
    // Must be called before robot.control(). mass<=0 -> leave the Desk-configured
    // load untouched (default).
    if (args.load_mass > 0.0) {
      // Non-fatal: a rejected load (e.g. an invalid inertia tensor) must not
      // brick the whole daemon.  Warn loudly and continue with whatever load
      // the robot already has (Desk-configured) so the operator notices the
      // camera is NOT compensated rather than losing the controller entirely.
      try {
        robot.setLoad(args.load_mass, args.load_com, args.load_inertia);
        std::cout << "[osc_shm] load     = " << args.load_mass << " kg, com=["
                  << args.load_com[0] << ", " << args.load_com[1] << ", "
                  << args.load_com[2] << "] m (gravity-compensated)" << std::endl;
      } catch (const franka::Exception& e) {
        std::cerr << "[osc_shm] WARN: setLoad FAILED (" << e.what()
                  << "). Continuing with the Desk-configured load -- the "
                     "payload is NOT compensated. Check load.mass/com/inertia "
                     "(inertia must be a valid positive-definite tensor)."
                  << std::endl;
      }
    } else {
      std::cout << "[osc_shm] load     = none (using Desk-configured load)"
                << std::endl;
    }

    const double ct = args.collision_torque;
    const double cc = args.collision_cartesian;
    robot.setCollisionBehavior(
        {{ct, ct, ct, ct, ct, ct, ct}},
        {{ct, ct, ct, ct, ct, ct, ct}},
        {{ct, ct, ct, ct, ct, ct, ct}},
        {{ct, ct, ct, ct, ct, ct, ct}},
        {{cc, cc, cc, cc, cc, cc}},
        {{cc, cc, cc, cc, cc, cc}},
        {{cc, cc, cc, cc, cc, cc}},
        {{cc, cc, cc, cc, cc, cc}});
    std::cout << "[osc_shm] collision = torque " << ct << " Nm, cartesian " << cc
              << " N/Nm (reflex thresholds)" << std::endl;

    franka::Model model = robot.loadModel();
    franka::RobotState initial_state = robot.readOnce();

    Eigen::Map<const Eigen::Matrix<double, 7, 1>> q_init(initial_state.q.data());
    if (!q_within_limits(q_init)) {
      // Downgraded from fatal (was: return 4) to a warning so the controller can
      // start from a config slightly outside the nominal band. libfranka's hard
      // joint limits still gate any actual motion.
      std::cerr << "[osc_shm] WARNING: q_init exceeds nominal Panda joint limits "
                   "(starting anyway)"
                << std::endl;
    }

    const PoseData anchor =
        extract_pose_data(model.pose(franka::Frame::kEndEffector, initial_state));

    // Seed the command with the anchor pose + default gains so that the
    // controller holds in place until a client overrides the setpoint.
    {
      ShmCommand* cmd = &shm->command;
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
      // Pure impedance to match the (unclipped) sim: 0 disables BOTH the
      // per-tick error clip (see the `err_dp > 0.0` guard where f_task is
      // built) AND the tracking-error abort further down. The remaining safety
      // net is the per-joint TAU_LIMIT clamp, the torque slew limiter, and
      // libfranka's own collision reflex + hard joint limits.
      // A client may still re-enable clipping at runtime via set_gains.
      cmd->error_delta_pos = 0.0;
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
    // Previous commanded torque, for the per-tick slew-rate limiter.  Starts at
    // zero: the robot enters control from rest, and the seeded setpoint
    // (anchor == current pose) yields ~zero torque, so ramping up from 0 is
    // both correct and smoother than the (unlimited) original first command.
    Eigen::Matrix<double, 7, 1> tau_prev = Eigen::Matrix<double, 7, 1>::Zero();
    // Controlled-stop state.  Once a stop is requested, `stopping` latches true:
    // subsequent ticks zero the pre-slew torque target so the slew limiter walks
    // tau_cmd down to zero, and `stop_ticks` bounds how long we wait to settle.
    bool stopping = false;
    size_t stop_ticks = 0;

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
      ShmCommand cmd_snap;
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

      Eigen::Vector3d e_pos = x_des - x;
      Eigen::Vector3d e_ori = shortest_quat_error_vec(q_des, q_cur);
      if (err_dp > 0.0) {
        e_pos = e_pos.cwiseMax(-err_dp).cwiseMin(err_dp);
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
      if (cmd_snap.enabled == 0u || stopping) {
        // Hold mode (enabled==0) or controlled stop (stopping): zero the pre-slew
        // command torque.  The slew limiter below then RAMPS tau_cmd toward 0
        // instead of stepping.  libfranka still adds gravity + friction comp.
        tau_pd.setZero();
      }
      Eigen::Matrix<double, 7, 1> tau_cmd =
          tau_pd.cwiseMax(-TAU_LIMIT).cwiseMin(TAU_LIMIT);

      // Torque slew-rate limiter.  The Jacobian-transpose impedance law emits a
      // STEP in commanded torque whenever its inputs jump discontinuously --
      // the setpoint (x_des/q_des) at a trajectory phase boundary, the gains
      // (set_gains), or enabled (enable/disable).  libfranka monitors the rate
      // of the commanded torque tau_J_d and aborts with the
      // "controller_torque_discontinuity" reflex when it exceeds kMaxTorqueRate
      // (1000 Nm/s).  Clamping each joint's per-tick change to
      // max_torque_rate*dt makes the torque RAMP to the new value over a few ms
      // instead of stepping, so the reflex cannot fire for ANY setpoint jump at
      // ANY configuration.  During normal in-phase motion dtau/dt is far below
      // the limit, so the limiter is inactive and does not alter the tracked
      // trajectory -- it only shapes the few-ms transitions at boundaries.
      if (args.max_torque_rate > 0.0) {
        double dt = period.toSec();
        if (dt <= 0.0) dt = 1e-3;  // libfranka's first callback reports period 0
        const double max_dtau = args.max_torque_rate * dt;
        for (int j = 0; j < 7; ++j) {
          const double lo = tau_prev(j) - max_dtau;
          const double hi = tau_prev(j) + max_dtau;
          if (tau_cmd(j) < lo) tau_cmd(j) = lo;
          else if (tau_cmd(j) > hi) tau_cmd(j) = hi;
        }
      }
      tau_prev = tau_cmd;

      // Publish the state frame to the ring buffer.
      ShmStateFrame frame{};
      frame.timestamp_s = monotonic_seconds() - t_start_mono;
      for (int j = 0; j < 7; ++j) {
        frame.q[j] = s.q[j];
        frame.dq[j] = s.dq[j];
        frame.tau[j] = tau_cmd(j);
        // Measured link-side torque (gravity included) -- this is the number
        // to compare against the 87/87/87/87/12/12/12 Nm joint limits.
        frame.tau_J[j] = s.tau_J[j];
      }
      frame.ee_pos[0] = x.x();
      frame.ee_pos[1] = x.y();
      frame.ee_pos[2] = x.z();
      frame.ee_quat[0] = q_cur.w();
      frame.ee_quat[1] = q_cur.x();
      frame.ee_quat[2] = q_cur.y();
      frame.ee_quat[3] = q_cur.z();
      // Measured EE Cartesian velocity (base frame) = zeroJacobian @ dq, already
      // computed above for the damping term. v = linear (m/s), w = angular (rad/s).
      frame.ee_linvel[0] = v.x();
      frame.ee_linvel[1] = v.y();
      frame.ee_linvel[2] = v.z();
      frame.ee_angvel[0] = w.x();
      frame.ee_angvel[1] = w.y();
      frame.ee_angvel[2] = w.z();
      panda_shm::state_publish(&shm->header, shm->states, frame,
                               FRANKATWIN_SHM_STATE_FRAMES);

      // Periodic diagnostic print (off by default).
      if (args.print_every > 0 &&
          tick % static_cast<size_t>(args.print_every) == 0) {
        std::cout << "[osc_shm] t=" << std::fixed << std::setprecision(3)
                  << elapsed << " enabled=" << cmd_snap.enabled
                  << " kp_pos=" << kp_pos << " |e_pos|_inf="
                  << std::setprecision(6) << e_pos.cwiseAbs().maxCoeff()
                  << " ||e_o||=" << e_ori.norm() << std::endl;
      }

      // Safety checks.
      // Per-tick nominal joint-limit abort downgraded to a one-shot warning: an
      // out-of-nominal joint no longer aborts the 1 kHz loop. libfranka still
      // enforces the robot's hard joint limits, so genuinely dangerous motion is
      // still stopped at the driver level.
      if (!q_within_limits(q)) {
        static bool warned_qlim = false;
        if (!warned_qlim) {
          warned_qlim = true;
          for (int j = 0; j < 7; ++j) {
            if (q(j) < Q_MIN(j) || q(j) > Q_MAX(j)) {
              std::cerr << "[osc_shm] WARNING: joint " << (j + 1)
                        << " out of nominal limits (q=" << q(j)
                        << "), not aborting" << std::endl;
              break;
            }
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

      const bool time_up = (args.duration > 0.0 && elapsed >= args.duration);
      ++tick;

      // Latch into the controlled-stop ramp on any stop cause.  We do NOT return
      // MotionFinished here: `stopping` makes the NEXT tick zero the pre-slew
      // torque target (above) so the slew limiter walks tau_cmd to 0 over a few
      // ms.  Hard-returning zero now would be an un-slewed STEP from the last
      // commanded torque -> controller_torque_discontinuity reflex, and declaring
      // the motion finished while the arm still moves -> "robot is still moving".
      if (g_stop_flag.load() || time_up || abort_code != 0) {
        stopping = true;
      }

      std::array<double, 7> tau_out{};
      Eigen::Map<Eigen::Matrix<double, 7, 1>>(tau_out.data()) = tau_cmd;

      if (stopping) {
        ++stop_ticks;
        const bool tau_settled = tau_cmd.cwiseAbs().maxCoeff() <= TAU_SETTLE_EPS;
        const bool vel_settled = dq.cwiseAbs().maxCoeff() <= DQ_SETTLE_EPS;
        // Finish only once the command torque has ramped to ~0 AND the arm is at
        // rest, so neither the torque-rate reflex nor the still-moving check
        // fires.  STOP_MAX_TICKS is a hard cap: by then the slew has long driven
        // tau to ~0, so finishing is discontinuity-safe even if velocity lingers.
        if ((tau_settled && vel_settled) || stop_ticks >= STOP_MAX_TICKS) {
          return franka::MotionFinished(franka::Torques(tau_out));
        }
      }

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
    } else {
      std::cout << "abort    : none" << std::endl;
    }
  } catch (const franka::Exception& e) {
    std::cerr << "[osc_shm] franka::Exception: " << e.what() << std::endl;
    __atomic_store_n(&shm->header.controller_pid, 0ull, __ATOMIC_RELEASE);
    munmap(shm, sizeof(ShmSegment));
    ::close(shm_fd);
    return 10;
  } catch (const std::exception& e) {
    std::cerr << "[osc_shm] std::exception: " << e.what() << std::endl;
    __atomic_store_n(&shm->header.controller_pid, 0ull, __ATOMIC_RELEASE);
    munmap(shm, sizeof(ShmSegment));
    ::close(shm_fd);
    return 11;
  }

  __atomic_store_n(&shm->header.controller_pid, 0ull, __ATOMIC_RELEASE);
  munmap(shm, sizeof(ShmSegment));
  ::close(shm_fd);
  // NOTE: we do NOT shm_unlink here. The owner of the segment (typically the
  // daemon or whoever launched us with --init-shm) is responsible for cleanup.
  return 0;
}
