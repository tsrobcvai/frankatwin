// step1_joint_pd.cpp
//
// Joint-space PD hold at 1 kHz, single-file standalone.
//     tau = Kp * (q_des - q) - Kd * dq
//
// Purpose of this step (not the OSC controller yet):
//   - validate the 1 kHz libfranka callback path
//   - validate SCHED_FIFO real-time priority on this machine
//   - measure dq (joint velocity) noise floor while holding still
//
// Safety:
//   - q_des is taken from --q-des on the CLI. The robot MUST already be at
//     (or extremely close to) q_des before this program is started.
//     We hard-fail if |q_init - q_des|_inf > Q_SAFETY_DELTA_RAD.
//   - Kp is ramped from 0 to its target value over --ramp seconds, so even
//     if you misjudge the initial pose the resulting torque grows gradually.
//   - Per-joint torque output is clamped to Franka's nominal motor limits.
//
// CLI:
//   ./step1_joint_pd <robot_ip>
//       --q-des q1 q2 q3 q4 q5 q6 q7    (required, 7 floats in radians)
//       [--kp K]          scalar broadcast to 7 joints     (default 50.0)
//       [--kd K]          scalar broadcast to 7 joints     (default 2*sqrt(kp))
//       [--duration sec]  total run time, 0 = until Ctrl+C (default 30.0)
//       [--ramp sec]      Kp ramp-in duration              (default 1.5)
//       [--log path]      CSV log path (omit -> no log)
//
// Build / run: see ../README.md and ../scripts/run_step1.sh.

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/robot.h>

#include <Eigen/Dense>

#include <array>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <pthread.h>
#include <sched.h>
#include <string>

namespace {

// Franka motor torque limits (Nm). joints 1-4 large, 5-7 small.
const Eigen::Matrix<double, 7, 1> TAU_LIMIT =
    (Eigen::Matrix<double, 7, 1>() << 87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0)
        .finished();

// Panda nominal joint position limits (rad).
const Eigen::Matrix<double, 7, 1> Q_MIN =
    (Eigen::Matrix<double, 7, 1>() << -2.8973, -1.7628, -2.8973, -3.0718,
     -2.8973, -0.0175, -2.8973)
        .finished();
const Eigen::Matrix<double, 7, 1> Q_MAX =
    (Eigen::Matrix<double, 7, 1>() << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973,
     3.7525, 2.8973)
        .finished();

// Largest allowed initial mismatch between robot's q and the requested q_des.
// If exceeded we refuse to run; the operator must move the robot first.
constexpr double Q_SAFETY_DELTA_RAD = 0.10;  // ~5.7 deg per joint

// Global stop flag for SIGINT / SIGTERM.
std::atomic<bool> g_stop_flag{false};
void signal_handler(int /*signo*/) { g_stop_flag.store(true); }

// ---------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------

struct Args {
  std::string robot_ip;
  Eigen::Matrix<double, 7, 1> q_des{Eigen::Matrix<double, 7, 1>::Zero()};
  bool q_des_set{false};
  double kp_scalar{50.0};
  bool kd_scalar_set{false};
  double kd_scalar{0.0};
  double duration{30.0};
  double ramp{1.5};
  std::string log_path;
};

void print_usage(const char *prog) {
  std::cerr << "Usage: " << prog
            << " <robot_ip> --q-des q1 q2 q3 q4 q5 q6 q7"
            << " [--kp K] [--kd K] [--duration sec] [--ramp sec] [--log path]"
            << std::endl;
}

bool parse_args(int argc, char **argv, Args &out) {
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
    if (key == "--q-des") {
      if (i + 7 >= argc) {
        std::cerr << "[step1] --q-des needs 7 floats" << std::endl;
        return false;
      }
      for (int j = 0; j < 7; ++j) {
        out.q_des(j) = std::atof(argv[i + 1 + j]);
      }
      out.q_des_set = true;
      i += 8;
    } else if (key == "--kp") {
      if (i + 1 >= argc) return false;
      out.kp_scalar = std::atof(argv[i + 1]);
      i += 2;
    } else if (key == "--kd") {
      if (i + 1 >= argc) return false;
      out.kd_scalar = std::atof(argv[i + 1]);
      out.kd_scalar_set = true;
      i += 2;
    } else if (key == "--duration") {
      if (i + 1 >= argc) return false;
      out.duration = std::atof(argv[i + 1]);
      i += 2;
    } else if (key == "--ramp") {
      if (i + 1 >= argc) return false;
      out.ramp = std::atof(argv[i + 1]);
      i += 2;
    } else if (key == "--log") {
      if (i + 1 >= argc) return false;
      out.log_path = argv[i + 1];
      i += 2;
    } else if (key == "-h" || key == "--help") {
      print_usage(argv[0]);
      return false;
    } else {
      std::cerr << "[step1] unknown arg: " << key << std::endl;
      print_usage(argv[0]);
      return false;
    }
  }

  if (!out.q_des_set) {
    std::cerr << "[step1] --q-des is required (7 floats, radians)" << std::endl;
    return false;
  }
  if (out.kp_scalar < 0.0 || out.kp_scalar > 2000.0) {
    std::cerr << "[step1] --kp out of range [0, 2000]" << std::endl;
    return false;
  }
  if (!out.kd_scalar_set) {
    out.kd_scalar = 2.0 * std::sqrt(out.kp_scalar);
  }
  if (out.ramp < 0.0) out.ramp = 0.0;
  return true;
}

// ---------------------------------------------------------------------------
// Real-time priority
// ---------------------------------------------------------------------------

// Returns true if SCHED_FIFO was successfully set. We do NOT exit on failure;
// the loop still runs (useful on non-RT kernels for smoke tests), but jitter
// numbers won't be representative of a real RT setup.
bool try_set_realtime_priority(int priority = 80) {
  sched_param sp{};
  sp.sched_priority = priority;
  int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
  if (rc != 0) {
    std::cerr << "[step1] WARN: failed to set SCHED_FIFO (rc=" << rc
              << ", errno=" << std::strerror(errno)
              << "). Continuing without RT priority. "
              << "Hint: ensure user is in 'realtime' group or run with "
                 "appropriate capabilities."
              << std::endl;
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// Jitter statistics (Welford online, ms units)
// ---------------------------------------------------------------------------

struct JitterStats {
  std::size_t n{0};
  double mean{0.0};
  double m2{0.0};  // for variance
  double min_v{std::numeric_limits<double>::infinity()};
  double max_v{-std::numeric_limits<double>::infinity()};

  void push(double x) {
    ++n;
    double delta = x - mean;
    mean += delta / static_cast<double>(n);
    double delta2 = x - mean;
    m2 += delta * delta2;
    if (x < min_v) min_v = x;
    if (x > max_v) max_v = x;
  }
  double std() const {
    return n > 1 ? std::sqrt(m2 / static_cast<double>(n - 1)) : 0.0;
  }
};

// ---------------------------------------------------------------------------
// Safety checks
// ---------------------------------------------------------------------------

bool q_within_limits(const Eigen::Matrix<double, 7, 1> &q) {
  for (int j = 0; j < 7; ++j) {
    if (q(j) < Q_MIN(j) || q(j) > Q_MAX(j)) return false;
  }
  return true;
}

}  // namespace

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char **argv) {
  Args args;
  if (!parse_args(argc, argv, args)) return 1;

  if (!q_within_limits(args.q_des)) {
    std::cerr << "[step1] q_des is outside Panda joint limits, aborting."
              << std::endl;
    return 2;
  }

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  const bool rt_ok = try_set_realtime_priority();

  Eigen::Matrix<double, 7, 1> kp_target =
      Eigen::Matrix<double, 7, 1>::Constant(args.kp_scalar);
  Eigen::Matrix<double, 7, 1> kd =
      Eigen::Matrix<double, 7, 1>::Constant(args.kd_scalar);

  std::cout << "[step1] robot_ip = " << args.robot_ip << "\n"
            << "[step1] q_des    = " << args.q_des.transpose() << "\n"
            << "[step1] kp       = " << args.kp_scalar
            << " (scalar, broadcast to 7)\n"
            << "[step1] kd       = " << args.kd_scalar
            << (args.kd_scalar_set ? " (user)" : " (auto = 2*sqrt(kp))") << "\n"
            << "[step1] ramp     = " << args.ramp << " s\n"
            << "[step1] duration = " << args.duration
            << " s (0 = until Ctrl+C)\n"
            << "[step1] RT       = " << (rt_ok ? "SCHED_FIFO" : "non-RT")
            << std::endl;

  std::ofstream log_file;
  bool log_enabled = false;
  if (!args.log_path.empty()) {
    log_file.open(args.log_path);
    if (!log_file.is_open()) {
      std::cerr << "[step1] cannot open log file: " << args.log_path
                << std::endl;
      return 3;
    }
    log_file << "t_s,period_ms";
    for (int j = 1; j <= 7; ++j) log_file << ",q" << j;
    for (int j = 1; j <= 7; ++j) log_file << ",dq" << j;
    for (int j = 1; j <= 7; ++j) log_file << ",tau" << j;
    log_file << "\n";
    log_enabled = true;
  }

  try {
    franka::Robot robot(args.robot_ip);

    // Wider thresholds than libfranka defaults so a small bump during
    // bring-up doesn't trip an immediate reflex. Tighten in later steps.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}});

    franka::RobotState initial_state = robot.readOnce();
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> q_init(initial_state.q.data());

    Eigen::Matrix<double, 7, 1> delta = (args.q_des - q_init).cwiseAbs();
    double delta_inf = delta.maxCoeff();
    std::cout << "[step1] q_init   = " << q_init.transpose() << "\n"
              << "[step1] |q_init - q_des|_inf = " << delta_inf << " rad"
              << std::endl;
    if (delta_inf > Q_SAFETY_DELTA_RAD) {
      std::cerr << "[step1] SAFETY: initial pose too far from q_des ("
                << delta_inf << " > " << Q_SAFETY_DELTA_RAD << " rad). "
                << "Move the robot to q_des first (e.g. using the desk "
                   "guiding mode), then re-run." << std::endl;
      return 4;
    }

    JitterStats jitter;
    Eigen::Matrix<double, 7, 1> dq_sq_sum =
        Eigen::Matrix<double, 7, 1>::Zero();
    std::size_t tick = 0;

    double elapsed = 0.0;

    auto callback = [&](const franka::RobotState &s,
                        franka::Duration period) -> franka::Torques {
      double dt = period.toSec();
      if (tick > 0) {
        jitter.push(period.toMSec());
      }
      elapsed += dt;

      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(s.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(s.dq.data());

      double alpha =
          (args.ramp > 0.0) ? std::min(1.0, elapsed / args.ramp) : 1.0;
      Eigen::Matrix<double, 7, 1> kp_eff = alpha * kp_target;

      Eigen::Matrix<double, 7, 1> tau =
          kp_eff.cwiseProduct(args.q_des - q) - kd.cwiseProduct(dq);
      tau = tau.cwiseMax(-TAU_LIMIT).cwiseMin(TAU_LIMIT);

      dq_sq_sum += dq.cwiseProduct(dq);

      if (log_enabled) {
        log_file << std::setprecision(6) << std::fixed << elapsed << ","
                 << period.toMSec();
        for (int j = 0; j < 7; ++j) log_file << "," << s.q[j];
        for (int j = 0; j < 7; ++j) log_file << "," << s.dq[j];
        for (int j = 0; j < 7; ++j) log_file << "," << tau(j);
        log_file << "\n";
      }

      ++tick;

      bool time_up = (args.duration > 0.0 && elapsed >= args.duration);
      if (g_stop_flag.load() || time_up) {
        std::array<double, 7> zero{};
        return franka::MotionFinished(franka::Torques(zero));
      }

      std::array<double, 7> tau_out{};
      Eigen::Map<Eigen::Matrix<double, 7, 1>>(tau_out.data()) = tau;
      return franka::Torques(tau_out);
    };

    std::cout << "[step1] starting 1 kHz torque control. Ctrl+C to stop."
              << std::endl;
    robot.control(callback);

    std::cout << "\n[step1] --- summary ---\n"
              << "RT priority           : " << (rt_ok ? "yes" : "no") << "\n"
              << "ticks                 : " << tick << "\n"
              << "elapsed (s)           : " << elapsed << "\n"
              << "period (ms) mean/std  : " << jitter.mean << " / "
              << jitter.std() << "\n"
              << "period (ms) min/max   : " << jitter.min_v << " / "
              << jitter.max_v << "\n";

    if (tick > 0) {
      Eigen::Matrix<double, 7, 1> dq_rms =
          (dq_sq_sum / static_cast<double>(tick)).cwiseSqrt();
      std::cout << "dq RMS (rad/s) per j  : " << dq_rms.transpose() << "\n";
    }
    if (log_enabled) {
      std::cout << "csv log               : " << args.log_path << "\n";
    }
    std::cout << std::flush;

  } catch (const franka::Exception &e) {
    std::cerr << "[step1] franka::Exception: " << e.what() << std::endl;
    return 10;
  } catch (const std::exception &e) {
    std::cerr << "[step1] std::exception: " << e.what() << std::endl;
    return 11;
  }

  return 0;
}
