// step5_cart_pd.cpp
//
// Jacobian-transpose Cartesian PD (position-only, no inertial decoupling)
// at 1 kHz, single-file standalone.
//
// Control law:
//   tau_cmd = J_p^T * (Kp * (x_des - x) - Kd * dx) + c(q, dq)
// where:
//   x   = EE position in base frame (3x1)
//   J_p = translational Jacobian (top 3 rows of zero Jacobian)
//   dx  = J_p * dq
//   c   = C(q,dq)*dq from franka::Model::coriolis()
//
// Effective robot-side torques:
//   tau_motor = tau_cmd + g(q) + tau_friction
//
// This step intentionally does NOT include:
//   - inertial decoupling (Lambda)
//   - orientation control
//   - nullspace posture control
//
// x_des trajectory:
//   - default hold: x_des = x_anchor (captured from readOnce at startup)
//   - optional single-axis sinusoid in base frame:
//       x_des[axis] = x_anchor[axis] + A_eff(t) * sin(2*pi*f*t)
//       dx_des[axis]= dA_eff/dt*sin(2*pi*f*t) + A_eff(t)*2*pi*f*cos(2*pi*f*t)
//   - dx_des is logged only (not used in damping term by design).
//
// Safety:
//   - q_init and runtime q are checked against Panda joint limits
//   - peak desired Cartesian speed check: amp*2*pi*freq <= 0.3 m/s
//   - runtime abort when |x_des - x|_inf exceeds 0.05 m
//
// CLI:
//   ./step5_cart_pd <robot_ip>
//       [--kp K]               scalar Cartesian stiffness      (default 100.0)
//       [--kd K]               scalar Cartesian damping        (default 2*sqrt(kp))
//       [--axis x|y|z]         sinusoid axis in base frame     (default z)
//       [--amp A]              sinusoid amplitude in meters    (default 0.0)
//       [--freq F]             sinusoid frequency in Hz        (default 0.25)
//       [--duration sec]       total runtime, 0=until Ctrl+C   (default 8.0)
//       [--ramp sec]           Kp ramp duration                (default 1.5)
//       [--amp-ramp sec]       amplitude ramp duration         (default --ramp)
//       [--no-coriolis]        disable explicit Coriolis       (default off)
//       [--print-err-every N]  print |x_des-x|_inf every N tick (default 100)
//       [--log path]           CSV path (omit -> no log)

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

constexpr double CART_DX_PEAK_LIMIT_MPS = 0.3;
constexpr double CART_TRACK_ABORT_M = 0.05;

std::atomic<bool> g_stop_flag{false};
void signal_handler(int /*signo*/) { g_stop_flag.store(true); }

enum Axis : int { kX = 0, kY = 1, kZ = 2 };

struct Args {
  std::string robot_ip;
  double kp_scalar{100.0};
  bool kd_scalar_set{false};
  double kd_scalar{0.0};
  Axis axis{kZ};
  double amp{0.0};
  double freq{0.25};
  double duration{8.0};
  double ramp{1.5};
  bool amp_ramp_set{false};
  double amp_ramp{0.0};
  bool no_coriolis{false};
  int print_err_every{100};
  std::string log_path;
};

const char* axis_to_string(Axis axis) {
  switch (axis) {
    case kX:
      return "x";
    case kY:
      return "y";
    case kZ:
      return "z";
  }
  return "?";
}

bool parse_axis(const std::string& s, Axis& axis) {
  if (s == "x" || s == "X") {
    axis = kX;
    return true;
  }
  if (s == "y" || s == "Y") {
    axis = kY;
    return true;
  }
  if (s == "z" || s == "Z") {
    axis = kZ;
    return true;
  }
  return false;
}

void print_usage(const char* prog) {
  std::cerr << "Usage: " << prog
            << " <robot_ip>"
            << " [--kp K] [--kd K] [--axis x|y|z] [--amp A] [--freq F]"
            << " [--duration sec] [--ramp sec] [--amp-ramp sec]"
            << " [--no-coriolis] [--print-err-every N] [--log path]"
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
    if (key == "--kp") {
      if (i + 1 >= argc) return false;
      out.kp_scalar = std::atof(argv[i + 1]);
      i += 2;
    } else if (key == "--kd") {
      if (i + 1 >= argc) return false;
      out.kd_scalar = std::atof(argv[i + 1]);
      out.kd_scalar_set = true;
      i += 2;
    } else if (key == "--axis") {
      if (i + 1 >= argc) return false;
      if (!parse_axis(argv[i + 1], out.axis)) {
        std::cerr << "[step5] --axis must be one of x|y|z" << std::endl;
        return false;
      }
      i += 2;
    } else if (key == "--amp") {
      if (i + 1 >= argc) return false;
      out.amp = std::atof(argv[i + 1]);
      i += 2;
    } else if (key == "--freq") {
      if (i + 1 >= argc) return false;
      out.freq = std::atof(argv[i + 1]);
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
      std::cerr << "[step5] unknown arg: " << key << std::endl;
      print_usage(argv[0]);
      return false;
    }
  }

  if (out.kp_scalar < 0.0 || out.kp_scalar > 4000.0) {
    std::cerr << "[step5] --kp out of range [0, 4000]" << std::endl;
    return false;
  }
  if (out.amp < 0.0 || out.amp > 0.20) {
    std::cerr << "[step5] --amp out of range [0, 0.20] meters" << std::endl;
    return false;
  }
  if (out.freq <= 0.0 || out.freq > 3.0) {
    std::cerr << "[step5] --freq out of range (0, 3.0] Hz" << std::endl;
    return false;
  }
  if (!out.kd_scalar_set) {
    out.kd_scalar = 2.0 * std::sqrt(out.kp_scalar);
  }
  if (out.print_err_every < 0) {
    std::cerr << "[step5] --print-err-every must be >= 0" << std::endl;
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
    std::cerr << "[step5] WARN: failed to set SCHED_FIFO (rc=" << rc
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

Eigen::Vector3d extract_position_from_pose(
    const std::array<double, 16>& pose_col_major) {
  return Eigen::Vector3d(pose_col_major[12], pose_col_major[13],
                         pose_col_major[14]);
}

}  // namespace

int main(int argc, char** argv) {
  Args args;
  if (!parse_args(argc, argv, args)) return 1;

  const double omega = 2.0 * M_PI * args.freq;
  const double dx_des_peak = args.amp * omega;
  if (dx_des_peak > CART_DX_PEAK_LIMIT_MPS) {
    std::cerr << "[step5] trajectory too fast: peak |dx_des| = " << dx_des_peak
              << " m/s > " << CART_DX_PEAK_LIMIT_MPS << " m/s" << std::endl;
    return 2;
  }

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
  const bool rt_ok = try_set_realtime_priority();

  const Eigen::Vector3d kp_target =
      Eigen::Vector3d::Constant(args.kp_scalar);
  const Eigen::Vector3d kd = Eigen::Vector3d::Constant(args.kd_scalar);

  std::cout << "[step5] robot_ip = " << args.robot_ip << "\n"
            << "[step5] axis     = " << axis_to_string(args.axis) << "\n"
            << "[step5] amp/freq = " << args.amp << " m, " << args.freq
            << " Hz\n"
            << "[step5] dx_des_peak = " << dx_des_peak << " m/s\n"
            << "[step5] kp       = " << args.kp_scalar
            << " (scalar, broadcast to xyz)\n"
            << "[step5] kd       = " << args.kd_scalar
            << (args.kd_scalar_set ? " (user)" : " (auto = 2*sqrt(kp))") << "\n"
            << "[step5] ramp     = " << args.ramp << " s\n"
            << "[step5] amp_ramp = " << args.amp_ramp << " s\n"
            << "[step5] duration = " << args.duration
            << " s (0 = until Ctrl+C)\n"
            << "[step5] coriolis = "
            << (args.no_coriolis ? "disabled (--no-coriolis)" : "enabled")
            << "\n"
            << "[step5] print-err-every = " << args.print_err_every
            << " ticks (0=off)\n"
            << "[step5] RT       = " << (rt_ok ? "SCHED_FIFO" : "non-RT")
            << std::endl;

  std::ofstream log_file;
  bool log_enabled = false;
  if (!args.log_path.empty()) {
    log_file.open(args.log_path);
    if (!log_file.is_open()) {
      std::cerr << "[step5] cannot open log file: " << args.log_path
                << std::endl;
      return 3;
    }
    log_file << "t_s,period_ms";
    for (int j = 1; j <= 7; ++j) log_file << ",q" << j;
    for (int j = 1; j <= 7; ++j) log_file << ",dq" << j;
    log_file << ",x_x,x_y,x_z";
    log_file << ",dx_x,dx_y,dx_z";
    log_file << ",x_des_x,x_des_y,x_des_z";
    log_file << ",dx_des_x,dx_des_y,dx_des_z";
    log_file << ",e_x,e_y,e_z";
    for (int j = 1; j <= 7; ++j) log_file << ",tau" << j;
    for (int j = 1; j <= 7; ++j) log_file << ",c" << j;
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
    if (!q_within_limits(q_init)) {
      std::cerr << "[step5] q_init exceeds nominal Panda joint limits."
                << std::endl;
      return 4;
    }

    const Eigen::Vector3d x_anchor =
        extract_position_from_pose(model.pose(franka::Frame::kEndEffector,
                                              initial_state));
    std::cout << "[step5] q_init   = " << q_init.transpose() << "\n"
              << "[step5] x_anchor = " << x_anchor.transpose() << " m"
              << std::endl;

    JitterStats jitter;
    Eigen::Matrix<double, 7, 1> dq_sq_sum =
        Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> c_sq_sum =
        Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Vector3d err_sq_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d err_abs_max = Eigen::Vector3d::Zero();
    double err_inf_max = 0.0;
    std::size_t tick = 0;
    double elapsed = 0.0;

    int abort_code = 0;  // 0 none, 1 cart tracking error, 2 joint limit
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

      const std::array<double, 16> pose_arr =
          model.pose(franka::Frame::kEndEffector, s);
      const Eigen::Vector3d x = extract_position_from_pose(pose_arr);

      const std::array<double, 42> jac_arr =
          model.zeroJacobian(franka::Frame::kEndEffector, s);
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jac6x7(jac_arr.data());
      const Eigen::Matrix<double, 3, 7> j_p = jac6x7.topRows<3>();
      const Eigen::Vector3d dx = j_p * dq;

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

      Eigen::Vector3d x_des = x_anchor;
      x_des(static_cast<int>(args.axis)) += a_eff * s_omega_t;
      Eigen::Vector3d dx_des = Eigen::Vector3d::Zero();
      dx_des(static_cast<int>(args.axis)) =
          da_eff * s_omega_t + a_eff * omega * c_omega_t;

      const Eigen::Vector3d kp_eff = alpha_kp * kp_target;
      const Eigen::Vector3d e = x_des - x;
      const Eigen::Vector3d f_task =
          kp_eff.cwiseProduct(e) - kd.cwiseProduct(dx);

      Eigen::Matrix<double, 7, 1> c_vec = Eigen::Matrix<double, 7, 1>::Zero();
      if (!args.no_coriolis) {
        const std::array<double, 7> c_arr = model.coriolis(s);
        c_vec = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(c_arr.data());
      }

      Eigen::Matrix<double, 7, 1> tau = j_p.transpose() * f_task + c_vec;
      tau = tau.cwiseMax(-TAU_LIMIT).cwiseMin(TAU_LIMIT);

      dq_sq_sum += dq.cwiseProduct(dq);
      c_sq_sum += c_vec.cwiseProduct(c_vec);
      err_sq_sum += e.cwiseProduct(e);
      err_abs_max = err_abs_max.cwiseMax(e.cwiseAbs());
      const double e_inf = e.cwiseAbs().maxCoeff();
      err_inf_max = std::max(err_inf_max, e_inf);

      if (args.print_err_every > 0 &&
          tick % static_cast<std::size_t>(args.print_err_every) == 0) {
        Eigen::Index max_idx = 0;
        const double err_inf = e.cwiseAbs().maxCoeff(&max_idx);
        std::cout << "[step5] t=" << std::fixed << std::setprecision(3)
                  << elapsed << " s"
                  << " |e|_inf=" << std::setprecision(6) << err_inf
                  << " m"
                  << " (" << (max_idx == 0 ? "x" : (max_idx == 1 ? "y" : "z"))
                  << ")" << std::endl;
      }

      if (log_enabled) {
        log_file << std::setprecision(6) << std::fixed << elapsed << ","
                 << period.toMSec();
        for (int j = 0; j < 7; ++j) log_file << "," << s.q[j];
        for (int j = 0; j < 7; ++j) log_file << "," << s.dq[j];
        log_file << "," << x(0) << "," << x(1) << "," << x(2);
        log_file << "," << dx(0) << "," << dx(1) << "," << dx(2);
        log_file << "," << x_des(0) << "," << x_des(1) << "," << x_des(2);
        log_file << "," << dx_des(0) << "," << dx_des(1) << "," << dx_des(2);
        log_file << "," << e(0) << "," << e(1) << "," << e(2);
        for (int j = 0; j < 7; ++j) log_file << "," << tau(j);
        for (int j = 0; j < 7; ++j) log_file << "," << c_vec(j);
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
      if (abort_code == 0 && e_inf > CART_TRACK_ABORT_M) {
        abort_code = 1;
        abort_time = elapsed;
        abort_value = e_inf;
      }

      if (g_stop_flag.load() || time_up || abort_code != 0) {
        std::array<double, 7> zero{};
        return franka::MotionFinished(franka::Torques(zero));
      }

      std::array<double, 7> tau_out{};
      Eigen::Map<Eigen::Matrix<double, 7, 1>>(tau_out.data()) = tau;
      return franka::Torques(tau_out);
    };

    std::cout << "[step5] starting 1 kHz torque control. Ctrl+C to stop."
              << std::endl;
    robot.control(callback);

    std::cout << "\n[step5] --- summary ---\n"
              << "RT priority                : " << (rt_ok ? "yes" : "no")
              << "\n"
              << "ticks                      : " << tick << "\n"
              << "elapsed (s)                : " << elapsed << "\n"
              << "period (ms) mean/std       : " << jitter.mean << " / "
              << jitter.std() << "\n"
              << "period (ms) min/max        : " << jitter.min_v << " / "
              << jitter.max_v << "\n";

    if (tick > 0) {
      const Eigen::Matrix<double, 7, 1> dq_rms =
          (dq_sq_sum / static_cast<double>(tick)).cwiseSqrt();
      const Eigen::Matrix<double, 7, 1> c_rms =
          (c_sq_sum / static_cast<double>(tick)).cwiseSqrt();
      const Eigen::Vector3d err_rms =
          (err_sq_sum / static_cast<double>(tick)).cwiseSqrt();
      std::cout << "dq RMS (rad/s) per j       : " << dq_rms.transpose() << "\n"
                << "c  RMS (Nm) per j          : " << c_rms.transpose() << "\n"
                << "cart err RMS (m) xyz       : " << err_rms.transpose() << "\n"
                << "cart err |max| (m) xyz     : " << err_abs_max.transpose()
                << "\n"
                << "cart |e|_inf max (m)       : " << err_inf_max << "\n";
    }
    if (abort_code == 1) {
      std::cout << "abort                      : cart tracking error at t="
                << abort_time << " s, |e|_inf=" << abort_value
                << " m (threshold " << CART_TRACK_ABORT_M << ")\n";
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
    std::cerr << "[step5] franka::Exception: " << e.what() << std::endl;
    return 10;
  } catch (const std::exception& e) {
    std::cerr << "[step5] std::exception: " << e.what() << std::endl;
    return 11;
  }

  return 0;
}
