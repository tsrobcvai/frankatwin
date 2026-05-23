// step4_joint_traj.cpp
//
// Joint-space trajectory tracking at 1 kHz, single-file standalone.
//
// Control law (same form as step 3, but q_des becomes time-varying):
//     tau_cmd(t) = Kp * (q_des(t) - q) - Kd * dq + c(q, dq)
// where c(q, dq) = C(q, dq) * dq from franka::Model::coriolis().
//
// Effective control law on robot:
//     tau_motor = tau_cmd + g(q) + tau_friction
//
// libfranka automatically adds gravity and friction compensation in torque mode.
// We add explicit Coriolis in this step unless --no-coriolis is specified.
//
// Trajectory shape:
//   - single-joint sinusoid around q_center
//   - selected joint j:
//       q_des_j(t) = q_center_j + A_eff(t) * sin(2*pi*f*t)
//       dq_des_j(t)= dA_eff/dt*sin(2*pi*f*t) + A_eff(t)*2*pi*f*cos(2*pi*f*t)
//   - other joints remain at q_center
//
// Safety additions over step 3:
//   - endpoint limit check for q_center +/- amp on selected joint
//   - peak desired-velocity check: amp*2*pi*freq <= 0.8 * joint_nominal_limit
//   - runtime abort if |q - q_des|_inf exceeds Q_TRACK_ABORT_RAD
//
// CLI:
//   ./step4_joint_traj <robot_ip>
//       --q-center q1 q2 q3 q4 q5 q6 q7   (required, radians)
//       [--joint J]            0..6 selected sweep joint       (default 3)
//       [--amp A]              sinusoid amplitude, rad          (default 0.10)
//       [--freq F]             sinusoid frequency, Hz           (default 0.25)
//       [--kp K]               scalar broadcast to 7 joints     (default 50.0)
//       [--kd K]               scalar broadcast to 7 joints     (default 2*sqrt(kp))
//       [--duration sec]       total runtime, 0=until Ctrl+C    (default 8.0)
//       [--ramp sec]           Kp ramp duration                 (default 1.5)
//       [--amp-ramp sec]       amplitude ramp duration          (default --ramp)
//       [--no-coriolis]        disable explicit Coriolis        (default off)
//       [--print-err-every N]  print tracking error every N tick (default 100)
//       [--log path]           CSV path (omit -> no log)
//
// Build / run: see ../README.md and ../scripts/run_step4.sh.

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
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

// Conservative nominal velocity limits used for pre-flight trajectory checks.
const Eigen::Matrix<double, 7, 1> DQ_NOMINAL_LIMIT =
    (Eigen::Matrix<double, 7, 1>() << 2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5)
        .finished();
constexpr double DQ_LIMIT_SCALE = 0.8;

constexpr double Q_SAFETY_DELTA_RAD = 0.25;
constexpr double Q_TRACK_ABORT_RAD = 0.35;

std::atomic<bool> g_stop_flag{false};
void signal_handler(int /*signo*/) { g_stop_flag.store(true); }

struct Args {
  std::string robot_ip;
  Eigen::Matrix<double, 7, 1> q_center{Eigen::Matrix<double, 7, 1>::Zero()};
  bool q_center_set{false};
  int joint{3};
  double amp{0.10};
  double freq{0.25};
  double kp_scalar{50.0};
  bool kd_scalar_set{false};
  double kd_scalar{0.0};
  double duration{8.0};
  double ramp{1.5};
  bool amp_ramp_set{false};
  double amp_ramp{0.0};
  bool no_coriolis{false};
  int print_err_every{100};
  std::string log_path;
};

void print_usage(const char* prog) {
  std::cerr << "Usage: " << prog
            << " <robot_ip> --q-center q1 q2 q3 q4 q5 q6 q7"
            << " [--joint J] [--amp A] [--freq F]"
            << " [--kp K] [--kd K] [--duration sec] [--ramp sec]"
            << " [--amp-ramp sec] [--no-coriolis]"
            << " [--print-err-every N] [--log path]" << std::endl;
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
    if (key == "--q-center") {
      if (i + 7 >= argc) {
        std::cerr << "[step4] --q-center needs 7 floats" << std::endl;
        return false;
      }
      for (int j = 0; j < 7; ++j) {
        out.q_center(j) = std::atof(argv[i + 1 + j]);
      }
      out.q_center_set = true;
      i += 8;
    } else if (key == "--joint") {
      if (i + 1 >= argc) return false;
      out.joint = std::atoi(argv[i + 1]);
      i += 2;
    } else if (key == "--amp") {
      if (i + 1 >= argc) return false;
      out.amp = std::atof(argv[i + 1]);
      i += 2;
    } else if (key == "--freq") {
      if (i + 1 >= argc) return false;
      out.freq = std::atof(argv[i + 1]);
      i += 2;
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
    } else if (key == "--amp-ramp") {
      if (i + 1 >= argc) return false;
      out.amp_ramp = std::atof(argv[i + 1]);
      out.amp_ramp_set = true;
      i += 2;
    } else if (key == "--no-coriolis") {
      out.no_coriolis = true;
      i += 1;
    } else if (key == "--print-err-every") {
      if (i + 1 >= argc) return false;
      out.print_err_every = std::atoi(argv[i + 1]);
      i += 2;
    } else if (key == "--log") {
      if (i + 1 >= argc) return false;
      out.log_path = argv[i + 1];
      i += 2;
    } else if (key == "-h" || key == "--help") {
      print_usage(argv[0]);
      return false;
    } else {
      std::cerr << "[step4] unknown arg: " << key << std::endl;
      print_usage(argv[0]);
      return false;
    }
  }

  if (!out.q_center_set) {
    std::cerr << "[step4] --q-center is required (7 floats, radians)"
              << std::endl;
    return false;
  }
  if (out.joint < 0 || out.joint > 6) {
    std::cerr << "[step4] --joint must be in [0, 6]" << std::endl;
    return false;
  }
  if (out.amp < 0.0 || out.amp > 2.0) {
    std::cerr << "[step4] --amp out of range [0, 2.0] rad" << std::endl;
    return false;
  }
  if (out.freq <= 0.0 || out.freq > 5.0) {
    std::cerr << "[step4] --freq out of range (0, 5.0] Hz" << std::endl;
    return false;
  }
  if (out.kp_scalar < 0.0 || out.kp_scalar > 2000.0) {
    std::cerr << "[step4] --kp out of range [0, 2000]" << std::endl;
    return false;
  }
  if (!out.kd_scalar_set) {
    out.kd_scalar = 2.0 * std::sqrt(out.kp_scalar);
  }
  if (out.print_err_every < 0) {
    std::cerr << "[step4] --print-err-every must be >= 0" << std::endl;
    return false;
  }
  if (out.ramp < 0.0) out.ramp = 0.0;
  if (!out.amp_ramp_set) out.amp_ramp = out.ramp;
  if (out.amp_ramp < 0.0) out.amp_ramp = 0.0;
  return true;
}

bool try_set_realtime_priority(int priority = 80) {
  sched_param sp{};
  sp.sched_priority = priority;
  int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
  if (rc != 0) {
    std::cerr << "[step4] WARN: failed to set SCHED_FIFO (rc=" << rc
              << ", errno=" << std::strerror(errno)
              << "). Continuing without RT priority. "
              << "Hint: ensure user is in 'realtime' group or run with "
                 "appropriate capabilities."
              << std::endl;
    return false;
  }
  return true;
}

struct JitterStats {
  std::size_t n{0};
  double mean{0.0};
  double m2{0.0};
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

bool q_within_limits(const Eigen::Matrix<double, 7, 1>& q) {
  for (int j = 0; j < 7; ++j) {
    if (q(j) < Q_MIN(j) || q(j) > Q_MAX(j)) return false;
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  Args args;
  if (!parse_args(argc, argv, args)) return 1;

  Eigen::Matrix<double, 7, 1> q_lo = args.q_center;
  Eigen::Matrix<double, 7, 1> q_hi = args.q_center;
  q_lo(args.joint) -= args.amp;
  q_hi(args.joint) += args.amp;
  if (!q_within_limits(args.q_center) || !q_within_limits(q_lo) ||
      !q_within_limits(q_hi)) {
    std::cerr << "[step4] q_center or trajectory endpoints exceed joint limits."
              << std::endl;
    return 2;
  }

  const double omega = 2.0 * M_PI * args.freq;
  const double dq_des_peak = args.amp * omega;
  const double dq_allowed = DQ_LIMIT_SCALE * DQ_NOMINAL_LIMIT(args.joint);
  if (dq_des_peak > dq_allowed) {
    std::cerr << "[step4] trajectory too fast for selected joint: peak dq_des="
              << dq_des_peak << " rad/s > allowed " << dq_allowed
              << " rad/s (0.8 * nominal)" << std::endl;
    return 3;
  }

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  const bool rt_ok = try_set_realtime_priority();

  Eigen::Matrix<double, 7, 1> kp_target =
      Eigen::Matrix<double, 7, 1>::Constant(args.kp_scalar);
  Eigen::Matrix<double, 7, 1> kd =
      Eigen::Matrix<double, 7, 1>::Constant(args.kd_scalar);

  std::cout << "[step4] robot_ip = " << args.robot_ip << "\n"
            << "[step4] q_center = " << args.q_center.transpose() << "\n"
            << "[step4] joint    = " << args.joint << " (0-based)\n"
            << "[step4] amp/freq = " << args.amp << " rad, " << args.freq
            << " Hz\n"
            << "[step4] dq_des_peak = " << dq_des_peak << " rad/s\n"
            << "[step4] kp       = " << args.kp_scalar
            << " (scalar, broadcast to 7)\n"
            << "[step4] kd       = " << args.kd_scalar
            << (args.kd_scalar_set ? " (user)" : " (auto = 2*sqrt(kp))") << "\n"
            << "[step4] ramp     = " << args.ramp << " s\n"
            << "[step4] amp_ramp = " << args.amp_ramp << " s\n"
            << "[step4] duration = " << args.duration
            << " s (0 = until Ctrl+C)\n"
            << "[step4] coriolis = "
            << (args.no_coriolis ? "disabled (--no-coriolis)" : "enabled")
            << "\n"
            << "[step4] print-err-every = " << args.print_err_every
            << " ticks (0=off)\n"
            << "[step4] RT       = " << (rt_ok ? "SCHED_FIFO" : "non-RT")
            << std::endl;

  std::ofstream log_file;
  bool log_enabled = false;
  if (!args.log_path.empty()) {
    log_file.open(args.log_path);
    if (!log_file.is_open()) {
      std::cerr << "[step4] cannot open log file: " << args.log_path
                << std::endl;
      return 4;
    }
    log_file << "t_s,period_ms";
    for (int j = 1; j <= 7; ++j) log_file << ",q" << j;
    for (int j = 1; j <= 7; ++j) log_file << ",dq" << j;
    for (int j = 1; j <= 7; ++j) log_file << ",q_des" << j;
    for (int j = 1; j <= 7; ++j) log_file << ",dq_des" << j;
    for (int j = 1; j <= 7; ++j) log_file << ",tau" << j;
    for (int j = 1; j <= 7; ++j) log_file << ",c" << j;
    for (int j = 1; j <= 7; ++j) log_file << ",err" << j;
    log_file << "\n";
    log_enabled = true;
  }

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

    Eigen::Matrix<double, 7, 1> delta = (args.q_center - q_init).cwiseAbs();
    double delta_inf = delta.maxCoeff();
    std::cout << "[step4] q_init   = " << q_init.transpose() << "\n"
              << "[step4] |q_init - q_center|_inf = " << delta_inf << " rad"
              << std::endl;
    if (delta_inf > Q_SAFETY_DELTA_RAD) {
      std::cerr << "[step4] SAFETY: initial pose too far from q_center ("
                << delta_inf << " > " << Q_SAFETY_DELTA_RAD << " rad). "
                << "Move the robot to q_center first and re-run."
                << std::endl;
      return 5;
    }

    JitterStats jitter;
    Eigen::Matrix<double, 7, 1> dq_sq_sum =
        Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> c_sq_sum =
        Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> err_sq_sum =
        Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> err_abs_max =
        Eigen::Matrix<double, 7, 1>::Zero();
    std::size_t tick = 0;
    double elapsed = 0.0;

    int abort_code = 0;  // 0 none, 1 tracking error, 2 joint limit
    int abort_joint = -1;
    double abort_value = 0.0;
    double abort_time = 0.0;

    auto callback = [&](const franka::RobotState& s,
                        franka::Duration period) -> franka::Torques {
      double dt = period.toSec();
      if (tick > 0) {
        jitter.push(period.toMSec());
      }
      elapsed += dt;

      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(s.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(s.dq.data());

      const double alpha_kp =
          (args.ramp > 0.0) ? std::min(1.0, elapsed / args.ramp) : 1.0;
      const double alpha_amp =
          (args.amp_ramp > 0.0) ? std::min(1.0, elapsed / args.amp_ramp) : 1.0;
      const double a_eff = args.amp * alpha_amp;
      const double da_eff =
          (args.amp_ramp > 0.0 && elapsed < args.amp_ramp)
              ? (args.amp / args.amp_ramp)
              : 0.0;

      const double s_omega_t = std::sin(omega * elapsed);
      const double c_omega_t = std::cos(omega * elapsed);

      Eigen::Matrix<double, 7, 1> q_des = args.q_center;
      q_des(args.joint) += a_eff * s_omega_t;

      Eigen::Matrix<double, 7, 1> dq_des = Eigen::Matrix<double, 7, 1>::Zero();
      dq_des(args.joint) = da_eff * s_omega_t + a_eff * omega * c_omega_t;

      Eigen::Matrix<double, 7, 1> c_vec = Eigen::Matrix<double, 7, 1>::Zero();
      if (!args.no_coriolis) {
        const std::array<double, 7> c_arr = model.coriolis(s);
        c_vec = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(c_arr.data());
      }

      Eigen::Matrix<double, 7, 1> kp_eff = alpha_kp * kp_target;
      Eigen::Matrix<double, 7, 1> err = q_des - q;
      Eigen::Matrix<double, 7, 1> tau =
          kp_eff.cwiseProduct(err) - kd.cwiseProduct(dq) + c_vec;
      tau = tau.cwiseMax(-TAU_LIMIT).cwiseMin(TAU_LIMIT);

      dq_sq_sum += dq.cwiseProduct(dq);
      c_sq_sum += c_vec.cwiseProduct(c_vec);
      err_sq_sum += err.cwiseProduct(err);
      err_abs_max = err_abs_max.cwiseMax(err.cwiseAbs());

      if (args.print_err_every > 0 &&
          tick % static_cast<std::size_t>(args.print_err_every) == 0) {
        Eigen::Index max_idx = 0;
        const double err_inf = err.cwiseAbs().maxCoeff(&max_idx);
        std::cout << "[step4] t=" << std::fixed << std::setprecision(3)
                  << elapsed << " s"
                  << " |err|_inf=" << std::setprecision(6) << err_inf
                  << " rad"
                  << " (j" << (max_idx + 1) << ")" << std::endl;
      }

      if (log_enabled) {
        log_file << std::setprecision(6) << std::fixed << elapsed << ","
                 << period.toMSec();
        for (int j = 0; j < 7; ++j) log_file << "," << s.q[j];
        for (int j = 0; j < 7; ++j) log_file << "," << s.dq[j];
        for (int j = 0; j < 7; ++j) log_file << "," << q_des(j);
        for (int j = 0; j < 7; ++j) log_file << "," << dq_des(j);
        for (int j = 0; j < 7; ++j) log_file << "," << tau(j);
        for (int j = 0; j < 7; ++j) log_file << "," << c_vec(j);
        for (int j = 0; j < 7; ++j) log_file << "," << err(j);
        log_file << "\n";
      }

      ++tick;

      bool time_up = (args.duration > 0.0 && elapsed >= args.duration);

      if (abort_code == 0 && !q_within_limits(q)) {
        abort_code = 2;
        abort_time = elapsed;
        for (int j = 0; j < 7; ++j) {
          if (q(j) < Q_MIN(j) || q(j) > Q_MAX(j)) {
            abort_joint = j;
            abort_value = q(j);
            break;
          }
        }
      }
      if (abort_code == 0) {
        Eigen::Index max_idx = 0;
        const double err_inf = err.cwiseAbs().maxCoeff(&max_idx);
        if (err_inf > Q_TRACK_ABORT_RAD) {
          abort_code = 1;
          abort_joint = static_cast<int>(max_idx);
          abort_value = err(abort_joint);
          abort_time = elapsed;
        }
      }

      if (g_stop_flag.load() || time_up || abort_code != 0) {
        std::array<double, 7> zero{};
        return franka::MotionFinished(franka::Torques(zero));
      }

      std::array<double, 7> tau_out{};
      Eigen::Map<Eigen::Matrix<double, 7, 1>>(tau_out.data()) = tau;
      return franka::Torques(tau_out);
    };

    std::cout << "[step4] starting 1 kHz torque control. Ctrl+C to stop."
              << std::endl;
    robot.control(callback);

    std::cout << "\n[step4] --- summary ---\n"
              << "RT priority                : " << (rt_ok ? "yes" : "no")
              << "\n"
              << "ticks                      : " << tick << "\n"
              << "elapsed (s)                : " << elapsed << "\n"
              << "period (ms) mean/std       : " << jitter.mean << " / "
              << jitter.std() << "\n"
              << "period (ms) min/max        : " << jitter.min_v << " / "
              << jitter.max_v << "\n";

    if (tick > 0) {
      Eigen::Matrix<double, 7, 1> dq_rms =
          (dq_sq_sum / static_cast<double>(tick)).cwiseSqrt();
      Eigen::Matrix<double, 7, 1> c_rms =
          (c_sq_sum / static_cast<double>(tick)).cwiseSqrt();
      Eigen::Matrix<double, 7, 1> err_rms =
          (err_sq_sum / static_cast<double>(tick)).cwiseSqrt();
      std::cout << "dq RMS (rad/s) per j       : " << dq_rms.transpose() << "\n"
                << "c  RMS (Nm) per j          : " << c_rms.transpose() << "\n"
                << "tracking err RMS (rad) per j: " << err_rms.transpose()
                << "\n"
                << "tracking err |max| (rad) per j: "
                << err_abs_max.transpose() << "\n";
    }
    if (abort_code == 1) {
      std::cout << "abort                      : tracking error at t="
                << abort_time << " s, joint " << (abort_joint + 1)
                << ", err=" << abort_value << " rad (threshold "
                << Q_TRACK_ABORT_RAD << ")\n";
    } else if (abort_code == 2) {
      std::cout << "abort                      : joint limit at t=" << abort_time
                << " s, joint " << (abort_joint + 1) << ", q=" << abort_value
                << " rad\n";
    } else {
      std::cout << "abort                      : none\n";
    }
    if (log_enabled) {
      std::cout << "csv log                    : " << args.log_path << "\n";
    }
    std::cout << std::flush;
  } catch (const franka::Exception& e) {
    std::cerr << "[step4] franka::Exception: " << e.what() << std::endl;
    return 10;
  } catch (const std::exception& e) {
    std::cerr << "[step4] std::exception: " << e.what() << std::endl;
    return 11;
  }

  return 0;
}
