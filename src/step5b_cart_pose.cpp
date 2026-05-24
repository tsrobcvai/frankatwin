// step5b_cart_pose.cpp
//
// Jacobian-transpose Cartesian PD (6D pose, no inertial decoupling / no Lambda)
// at 1 kHz, single-file standalone.
//
// Control law:
//   tau_cmd = J^T * F_task + c(q, dq)
//   F_task  = [ Kp_pos * (x_des - x) - Kd_pos * v ;
//               Kp_ori * e_o         - Kd_ori * w ]
// where:
//   x   = EE position in base frame
//   R   = EE orientation in base frame
//   J   = 6x7 zero Jacobian at EE frame (base coordinates)
//   v   = J_pos * dq
//   w   = J_ori * dq
//   e_o = 2 * vec(q_des * q^{-1}) with shortest-path sign handling
//   c   = C(q,dq)*dq from franka::Model::coriolis()
//
// Effective robot-side torques:
//   tau_motor = tau_cmd + g(q) + tau_friction
//
// This step intentionally does NOT include:
//   - inertial decoupling (Lambda)
//   - nullspace posture control
//
// Trajectory:
//   - position: optional single-axis sinusoid around x_anchor
//   - orientation: hold at R_anchor (captured at startup)
//
// Safety:
//   - q_init and runtime q are checked against Panda joint limits
//   - peak desired Cartesian speed check: amp*2*pi*freq <= 0.3 m/s
//   - runtime abort when |x_des - x|_inf exceeds 0.05 m
//   - runtime abort when ||e_o|| exceeds 0.30 rad
//
// CLI:
//   ./step5b_cart_pose <robot_ip>
//       [--kp-pos K]            scalar Cartesian pos stiffness (default 100.0)
//       [--kd-pos K]            scalar Cartesian pos damping   (default 2*sqrt(kp-pos))
//       [--kp-ori K]            scalar ori stiffness (default 20.0)
//       [--kd-ori K]            scalar ori damping   (default 2*sqrt(kp-ori))
//       [--axis x|y|z]          sinusoid axis in base frame (default z)
//       [--amp A]               sinusoid amplitude in meters (default 0.0)
//       [--freq F]              sinusoid frequency in Hz     (default 0.25)
//       [--duration sec]        total runtime, 0=until Ctrl+C (default 8.0)
//       [--ramp sec]            gain ramp duration            (default 1.5)
//       [--amp-ramp sec]        amplitude ramp duration       (default --ramp)
//       [--no-coriolis]         disable explicit Coriolis     (default off)
//       [--print-err-every N]   print |e_pos|_inf and ||e_o|| every N tick
//       [--log path]            CSV path (omit -> no log)
//       [--sidecar path]        JSON sidecar path (omit -> log path with .csv->.json)
//
// CSV columns (per tick):
//   t_s, period_ms,
//   q1..q7, dq1..dq7,
//   x_x,x_y,x_z, dx_x,dx_y,dx_z,
//   quat_x,quat_y,quat_z,quat_w, wx,wy,wz,
//   x_des_{x,y,z}, dx_des_{x,y,z},
//   quat_des_{x,y,z,w},
//   e_x,e_y,e_z, e_ox,e_oy,e_oz,
//   tau_pd1..tau_pd7      (pre-clamp computed torque = J^T*f_task + c_vec)
//   tau_cmd1..tau_cmd7    (post-clamp torque actually returned to libfranka)
//   c1..c7                (explicit Coriolis vector; 0 if --no-coriolis)
//
// JSON sidecar (written once at exit, success or exception):
//   schema_version, controller, started_utc, csv_path, sidecar_path,
//   robot_ip, control_rate_hz, frame, rt_priority,
//   args { kp_pos, kd_pos, kp_ori, kd_ori, axis, amp, freq,
//          ori_axis, ori_amp_deg, ori_freq, ori_frame,
//          duration, ramp, amp_ramp, no_coriolis, print_err_every },
//   q_init[7], x_anchor[3], q_anchor_xyzw[4],
//   abort { code, name, joint, value, time_s },
//   summary { ticks, elapsed_s, period_ms_*, dq_rms[7], c_rms[7],
//             err_pos_rms_xyz[3], err_pos_abs_max_xyz[3], err_pos_inf_max,
//             err_ori_rms_xyz[3], err_ori_abs_max_xyz[3], err_ori_norm_max },
//   exception, ended_normally
// Intent: sim replay can pick up q_init + x_anchor/q_anchor_xyzw and stream
// (t_s, x_des_*, quat_des_*) from the CSV; comparison plots use (x_*, quat_*).

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <pthread.h>
#include <sched.h>
#include <sstream>
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
constexpr double ORI_TRACK_ABORT_RAD = 0.30;
constexpr double ORI_DOT_PEAK_LIMIT_RPS = 0.5;

std::atomic<bool> g_stop_flag{false};
void signal_handler(int /*signo*/) { g_stop_flag.store(true); }

enum Axis : int { kX = 0, kY = 1, kZ = 2 };

struct Args {
  std::string robot_ip;
  double kp_pos{100.0};
  bool kd_pos_set{false};
  double kd_pos{0.0};
  double kp_ori{20.0};
  bool kd_ori_set{false};
  double kd_ori{0.0};
  Axis axis{kZ};
  double amp{0.0};
  double freq{0.25};
  Axis ori_axis{kZ};
  double ori_amp_deg{0.0};
  double ori_freq{0.25};
  bool ori_frame_ee{false};
  double duration{8.0};
  double ramp{1.5};
  bool amp_ramp_set{false};
  double amp_ramp{0.0};
  bool no_coriolis{false};
  int print_err_every{100};
  std::string log_path;
  std::string sidecar_path;
};

struct PoseData {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
  Eigen::Quaterniond quaternion{Eigen::Quaterniond::Identity()};
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
  std::cerr << "Usage: " << prog << " <robot_ip>"
            << " [--kp-pos K] [--kd-pos K] [--kp-ori K] [--kd-ori K]"
            << " [--axis x|y|z] [--amp A] [--freq F]"
            << " [--ori-axis x|y|z] [--ori-amp-deg DEG] [--ori-freq F]"
            << " [--ori-frame base|ee]"
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
    if (key == "--kp-pos") {
      if (i + 1 >= argc) return false;
      out.kp_pos = std::atof(argv[i + 1]);
      i += 2;
    } else if (key == "--kd-pos") {
      if (i + 1 >= argc) return false;
      out.kd_pos = std::atof(argv[i + 1]);
      out.kd_pos_set = true;
      i += 2;
    } else if (key == "--kp-ori") {
      if (i + 1 >= argc) return false;
      out.kp_ori = std::atof(argv[i + 1]);
      i += 2;
    } else if (key == "--kd-ori") {
      if (i + 1 >= argc) return false;
      out.kd_ori = std::atof(argv[i + 1]);
      out.kd_ori_set = true;
      i += 2;
    } else if (key == "--axis") {
      if (i + 1 >= argc) return false;
      if (!parse_axis(argv[i + 1], out.axis)) {
        std::cerr << "[step5b] --axis must be one of x|y|z" << std::endl;
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
    } else if (key == "--ori-axis") {
      if (i + 1 >= argc) return false;
      if (!parse_axis(argv[i + 1], out.ori_axis)) {
        std::cerr << "[step5b] --ori-axis must be one of x|y|z" << std::endl;
        return false;
      }
      i += 2;
    } else if (key == "--ori-amp-deg") {
      if (i + 1 >= argc) return false;
      out.ori_amp_deg = std::atof(argv[i + 1]);
      i += 2;
    } else if (key == "--ori-freq") {
      if (i + 1 >= argc) return false;
      out.ori_freq = std::atof(argv[i + 1]);
      i += 2;
    } else if (key == "--ori-frame") {
      if (i + 1 >= argc) return false;
      const std::string v = argv[i + 1];
      if (v == "base") {
        out.ori_frame_ee = false;
      } else if (v == "ee") {
        out.ori_frame_ee = true;
      } else {
        std::cerr << "[step5b] --ori-frame must be one of base|ee" << std::endl;
        return false;
      }
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
    } else if (key == "--sidecar") {
      if (i + 1 >= argc) return false;
      out.sidecar_path = argv[i + 1];
      i += 2;
    } else if (key == "-h" || key == "--help") {
      print_usage(argv[0]);
      return false;
    } else {
      std::cerr << "[step5b] unknown arg: " << key << std::endl;
      print_usage(argv[0]);
      return false;
    }
  }

  if (out.kp_pos < 0.0 || out.kp_pos > 4000.0) {
    std::cerr << "[step5b] --kp-pos out of range [0, 4000]" << std::endl;
    return false;
  }
  if (out.kp_ori < 0.0 || out.kp_ori > 200.0) {
    std::cerr << "[step5b] --kp-ori out of range [0, 200]" << std::endl;
    return false;
  }
  if (out.amp < 0.0 || out.amp > 0.20) {
    std::cerr << "[step5b] --amp out of range [0, 0.20] meters" << std::endl;
    return false;
  }
  if (out.freq <= 0.0 || out.freq > 3.0) {
    std::cerr << "[step5b] --freq out of range (0, 3.0] Hz" << std::endl;
    return false;
  }
  if (out.ori_amp_deg < 0.0 || out.ori_amp_deg > 30.0) {
    std::cerr << "[step5b] --ori-amp-deg out of range [0, 30] deg" << std::endl;
    return false;
  }
  if (out.ori_freq <= 0.0 || out.ori_freq > 3.0) {
    std::cerr << "[step5b] --ori-freq out of range (0, 3.0] Hz" << std::endl;
    return false;
  }
  if (!out.kd_pos_set) {
    out.kd_pos = 2.0 * std::sqrt(out.kp_pos);
  }
  if (!out.kd_ori_set) {
    out.kd_ori = 2.0 * std::sqrt(out.kp_ori);
  }
  if (out.print_err_every < 0) {
    std::cerr << "[step5b] --print-err-every must be >= 0" << std::endl;
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
    std::cerr << "[step5b] WARN: failed to set SCHED_FIFO (rc=" << rc
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

PoseData extract_pose_data(const std::array<double, 16>& pose_col_major) {
  PoseData pose;
  pose.position = Eigen::Vector3d(pose_col_major[12], pose_col_major[13],
                                  pose_col_major[14]);
  pose.rotation << pose_col_major[0], pose_col_major[4], pose_col_major[8],
      pose_col_major[1], pose_col_major[5], pose_col_major[9], pose_col_major[2],
      pose_col_major[6], pose_col_major[10];
  pose.quaternion = Eigen::Quaterniond(pose.rotation);
  pose.quaternion.normalize();
  return pose;
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

std::string default_sidecar_from_log(const std::string& log_path) {
  if (log_path.empty()) return std::string();
  const std::string suffix = ".csv";
  if (log_path.size() >= suffix.size() &&
      log_path.compare(log_path.size() - suffix.size(), suffix.size(),
                       suffix) == 0) {
    return log_path.substr(0, log_path.size() - suffix.size()) + ".json";
  }
  return log_path + ".json";
}

std::string utc_iso8601_now() {
  const std::time_t t = std::time(nullptr);
  std::tm gm{};
#if defined(_WIN32)
  gmtime_s(&gm, &t);
#else
  gmtime_r(&t, &gm);
#endif
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &gm);
  return std::string(buf);
}

std::string json_escape(const std::string& s) {
  std::string out;
  out.reserve(s.size() + 2);
  for (char c : s) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\b': out += "\\b"; break;
      case '\f': out += "\\f"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<int>(c));
          out += buf;
        } else {
          out += c;
        }
    }
  }
  return out;
}

std::string json_double(double x) {
  if (std::isnan(x) || std::isinf(x)) return "null";
  std::ostringstream oss;
  oss << std::setprecision(17) << x;
  return oss.str();
}

std::string json_double_array(const double* arr, int n) {
  std::ostringstream oss;
  oss << "[";
  for (int i = 0; i < n; ++i) {
    if (i > 0) oss << ", ";
    oss << json_double(arr[i]);
  }
  oss << "]";
  return oss.str();
}

const char* abort_code_to_name(int code) {
  switch (code) {
    case 0: return "none";
    case 1: return "position_tracking_error";
    case 2: return "joint_limit";
    case 3: return "orientation_tracking_error";
    default: return "unknown";
  }
}

struct RunRecord {
  std::string controller{"step5b_cart_pose"};
  std::string started_utc;
  std::string csv_path;
  std::string sidecar_path;
  std::string robot_ip;
  int control_rate_hz{1000};
  std::string frame{"panda_link0 (base)"};
  bool rt_priority{false};
  Args args;

  bool has_init{false};
  std::array<double, 7> q_init{};
  std::array<double, 3> x_anchor{};
  std::array<double, 4> q_anchor_xyzw{};

  // abort populated from per-tick locals after control loop returns.
  int abort_code{0};
  int abort_joint{-1};
  double abort_value{0.0};
  double abort_time_s{0.0};

  bool has_summary{false};
  std::size_t ticks{0};
  double elapsed_s{0.0};
  double period_ms_mean{0.0};
  double period_ms_std{0.0};
  double period_ms_min{0.0};
  double period_ms_max{0.0};
  std::array<double, 7> dq_rms{};
  std::array<double, 7> c_rms{};
  std::array<double, 3> err_pos_rms{};
  std::array<double, 3> err_pos_abs_max{};
  double err_pos_inf_max{0.0};
  std::array<double, 3> err_ori_rms{};
  std::array<double, 3> err_ori_abs_max{};
  double err_ori_norm_max{0.0};

  bool had_exception{false};
  std::string exception_message;
  bool ended_normally{false};
};

void write_sidecar_json(const RunRecord& r) {
  if (r.sidecar_path.empty()) return;
  std::ofstream f(r.sidecar_path);
  if (!f.is_open()) {
    std::cerr << "[step5b] WARN: cannot open sidecar JSON for write: "
              << r.sidecar_path << std::endl;
    return;
  }

  f << "{\n";
  f << "  \"schema_version\": 1,\n";
  f << "  \"controller\": \"" << json_escape(r.controller) << "\",\n";
  f << "  \"started_utc\": \"" << json_escape(r.started_utc) << "\",\n";
  f << "  \"csv_path\": \"" << json_escape(r.csv_path) << "\",\n";
  f << "  \"sidecar_path\": \"" << json_escape(r.sidecar_path) << "\",\n";
  f << "  \"robot_ip\": \"" << json_escape(r.robot_ip) << "\",\n";
  f << "  \"control_rate_hz\": " << r.control_rate_hz << ",\n";
  f << "  \"frame\": \"" << json_escape(r.frame) << "\",\n";
  f << "  \"rt_priority\": " << (r.rt_priority ? "true" : "false") << ",\n";

  f << "  \"args\": {\n";
  f << "    \"kp_pos\": " << json_double(r.args.kp_pos) << ",\n";
  f << "    \"kd_pos\": " << json_double(r.args.kd_pos) << ",\n";
  f << "    \"kd_pos_user_set\": "
    << (r.args.kd_pos_set ? "true" : "false") << ",\n";
  f << "    \"kp_ori\": " << json_double(r.args.kp_ori) << ",\n";
  f << "    \"kd_ori\": " << json_double(r.args.kd_ori) << ",\n";
  f << "    \"kd_ori_user_set\": "
    << (r.args.kd_ori_set ? "true" : "false") << ",\n";
  f << "    \"axis\": \"" << axis_to_string(r.args.axis) << "\",\n";
  f << "    \"amp\": " << json_double(r.args.amp) << ",\n";
  f << "    \"freq\": " << json_double(r.args.freq) << ",\n";
  f << "    \"ori_axis\": \"" << axis_to_string(r.args.ori_axis) << "\",\n";
  f << "    \"ori_amp_deg\": " << json_double(r.args.ori_amp_deg) << ",\n";
  f << "    \"ori_freq\": " << json_double(r.args.ori_freq) << ",\n";
  f << "    \"ori_frame\": \"" << (r.args.ori_frame_ee ? "ee" : "base")
    << "\",\n";
  f << "    \"duration\": " << json_double(r.args.duration) << ",\n";
  f << "    \"ramp\": " << json_double(r.args.ramp) << ",\n";
  f << "    \"amp_ramp\": " << json_double(r.args.amp_ramp) << ",\n";
  f << "    \"no_coriolis\": " << (r.args.no_coriolis ? "true" : "false")
    << ",\n";
  f << "    \"print_err_every\": " << r.args.print_err_every << "\n";
  f << "  },\n";

  if (r.has_init) {
    f << "  \"q_init\": " << json_double_array(r.q_init.data(), 7) << ",\n";
    f << "  \"x_anchor\": " << json_double_array(r.x_anchor.data(), 3) << ",\n";
    f << "  \"q_anchor_xyzw\": "
      << json_double_array(r.q_anchor_xyzw.data(), 4) << ",\n";
  } else {
    f << "  \"q_init\": null,\n";
    f << "  \"x_anchor\": null,\n";
    f << "  \"q_anchor_xyzw\": null,\n";
  }

  f << "  \"abort\": {\n";
  f << "    \"code\": " << r.abort_code << ",\n";
  f << "    \"name\": \"" << json_escape(abort_code_to_name(r.abort_code))
    << "\",\n";
  f << "    \"joint\": " << r.abort_joint << ",\n";
  f << "    \"value\": " << json_double(r.abort_value) << ",\n";
  f << "    \"time_s\": " << json_double(r.abort_time_s) << "\n";
  f << "  },\n";

  if (r.has_summary) {
    f << "  \"summary\": {\n";
    f << "    \"ticks\": " << r.ticks << ",\n";
    f << "    \"elapsed_s\": " << json_double(r.elapsed_s) << ",\n";
    f << "    \"period_ms_mean\": " << json_double(r.period_ms_mean) << ",\n";
    f << "    \"period_ms_std\": " << json_double(r.period_ms_std) << ",\n";
    f << "    \"period_ms_min\": " << json_double(r.period_ms_min) << ",\n";
    f << "    \"period_ms_max\": " << json_double(r.period_ms_max) << ",\n";
    f << "    \"dq_rms\": " << json_double_array(r.dq_rms.data(), 7) << ",\n";
    f << "    \"c_rms\": " << json_double_array(r.c_rms.data(), 7) << ",\n";
    f << "    \"err_pos_rms_xyz\": "
      << json_double_array(r.err_pos_rms.data(), 3) << ",\n";
    f << "    \"err_pos_abs_max_xyz\": "
      << json_double_array(r.err_pos_abs_max.data(), 3) << ",\n";
    f << "    \"err_pos_inf_max\": " << json_double(r.err_pos_inf_max) << ",\n";
    f << "    \"err_ori_rms_xyz\": "
      << json_double_array(r.err_ori_rms.data(), 3) << ",\n";
    f << "    \"err_ori_abs_max_xyz\": "
      << json_double_array(r.err_ori_abs_max.data(), 3) << ",\n";
    f << "    \"err_ori_norm_max\": " << json_double(r.err_ori_norm_max)
      << "\n";
    f << "  },\n";
  } else {
    f << "  \"summary\": null,\n";
  }

  if (r.had_exception) {
    f << "  \"exception\": \"" << json_escape(r.exception_message) << "\",\n";
  } else {
    f << "  \"exception\": null,\n";
  }

  f << "  \"ended_normally\": " << (r.ended_normally ? "true" : "false")
    << "\n";
  f << "}\n";
}

}  // namespace

int main(int argc, char** argv) {
  Args args;
  if (!parse_args(argc, argv, args)) return 1;

  if (args.sidecar_path.empty()) {
    args.sidecar_path = default_sidecar_from_log(args.log_path);
  }

  RunRecord record;
  record.started_utc = utc_iso8601_now();
  record.csv_path = args.log_path;
  record.sidecar_path = args.sidecar_path;
  record.robot_ip = args.robot_ip;
  record.args = args;

  const double omega = 2.0 * M_PI * args.freq;
  const double dx_des_peak = args.amp * omega;
  if (dx_des_peak > CART_DX_PEAK_LIMIT_MPS) {
    std::cerr << "[step5b] trajectory too fast: peak |dx_des| = " << dx_des_peak
              << " m/s > " << CART_DX_PEAK_LIMIT_MPS << " m/s" << std::endl;
    return 2;
  }

  const double omega_o = 2.0 * M_PI * args.ori_freq;
  const double ori_amp_rad = args.ori_amp_deg * M_PI / 180.0;
  const double w_des_peak = ori_amp_rad * omega_o;
  if (w_des_peak > ORI_DOT_PEAK_LIMIT_RPS) {
    std::cerr << "[step5b] orientation trajectory too fast: peak |w_des| = "
              << w_des_peak << " rad/s > " << ORI_DOT_PEAK_LIMIT_RPS << " rad/s"
              << std::endl;
    return 2;
  }

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
  const bool rt_ok = try_set_realtime_priority();
  record.rt_priority = rt_ok;

  const Eigen::Vector3d kp_pos_target = Eigen::Vector3d::Constant(args.kp_pos);
  const Eigen::Vector3d kd_pos = Eigen::Vector3d::Constant(args.kd_pos);
  const Eigen::Vector3d kp_ori_target = Eigen::Vector3d::Constant(args.kp_ori);
  const Eigen::Vector3d kd_ori = Eigen::Vector3d::Constant(args.kd_ori);

  std::cout << "[step5b] robot_ip = " << args.robot_ip << "\n"
            << "[step5b] axis     = " << axis_to_string(args.axis) << "\n"
            << "[step5b] amp/freq = " << args.amp << " m, " << args.freq
            << " Hz\n"
            << "[step5b] dx_des_peak = " << dx_des_peak << " m/s\n"
            << "[step5b] ori-axis = " << axis_to_string(args.ori_axis) << "\n"
            << "[step5b] ori-amp/freq = " << args.ori_amp_deg << " deg, "
            << args.ori_freq << " Hz\n"
            << "[step5b] ori-frame = " << (args.ori_frame_ee ? "ee" : "base")
            << "\n"
            << "[step5b] w_des_peak = " << w_des_peak << " rad/s\n"
            << "[step5b] kp_pos   = " << args.kp_pos << "\n"
            << "[step5b] kd_pos   = " << args.kd_pos
            << (args.kd_pos_set ? " (user)" : " (auto = 2*sqrt(kp_pos))") << "\n"
            << "[step5b] kp_ori   = " << args.kp_ori << "\n"
            << "[step5b] kd_ori   = " << args.kd_ori
            << (args.kd_ori_set ? " (user)" : " (auto = 2*sqrt(kp_ori))") << "\n"
            << "[step5b] ramp     = " << args.ramp << " s\n"
            << "[step5b] amp_ramp = " << args.amp_ramp << " s\n"
            << "[step5b] duration = " << args.duration
            << " s (0 = until Ctrl+C)\n"
            << "[step5b] coriolis = "
            << (args.no_coriolis ? "disabled (--no-coriolis)" : "enabled")
            << "\n"
            << "[step5b] print-err-every = " << args.print_err_every
            << " ticks (0=off)\n"
            << "[step5b] RT       = " << (rt_ok ? "SCHED_FIFO" : "non-RT")
            << "\n"
            << "[step5b] sidecar  = "
            << (args.sidecar_path.empty() ? std::string("(disabled)")
                                          : args.sidecar_path)
            << std::endl;

  std::ofstream log_file;
  bool log_enabled = false;
  if (!args.log_path.empty()) {
    log_file.open(args.log_path);
    if (!log_file.is_open()) {
      std::cerr << "[step5b] cannot open log file: " << args.log_path << std::endl;
      return 3;
    }
    log_file << "t_s,period_ms";
    for (int j = 1; j <= 7; ++j) log_file << ",q" << j;
    for (int j = 1; j <= 7; ++j) log_file << ",dq" << j;
    log_file << ",x_x,x_y,x_z";
    log_file << ",dx_x,dx_y,dx_z";
    log_file << ",quat_x,quat_y,quat_z,quat_w";
    log_file << ",wx,wy,wz";
    log_file << ",x_des_x,x_des_y,x_des_z";
    log_file << ",dx_des_x,dx_des_y,dx_des_z";
    log_file << ",quat_des_x,quat_des_y,quat_des_z,quat_des_w";
    log_file << ",e_x,e_y,e_z";
    log_file << ",e_ox,e_oy,e_oz";
    for (int j = 1; j <= 7; ++j) log_file << ",tau_pd" << j;
    for (int j = 1; j <= 7; ++j) log_file << ",tau_cmd" << j;
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
      std::cerr << "[step5b] q_init exceeds nominal Panda joint limits."
                << std::endl;
      return 4;
    }

    const PoseData anchor_pose =
        extract_pose_data(model.pose(franka::Frame::kEndEffector, initial_state));
    const Eigen::Vector3d x_anchor = anchor_pose.position;
    const Eigen::Quaterniond q_anchor = anchor_pose.quaternion;

    record.has_init = true;
    for (int j = 0; j < 7; ++j) record.q_init[j] = q_init(j);
    for (int i = 0; i < 3; ++i) record.x_anchor[i] = x_anchor(i);
    record.q_anchor_xyzw = {q_anchor.x(), q_anchor.y(), q_anchor.z(),
                            q_anchor.w()};

    std::cout << "[step5b] q_init   = " << q_init.transpose() << "\n"
              << "[step5b] x_anchor = " << x_anchor.transpose() << " m\n"
              << "[step5b] q_anchor (xyzw) = " << q_anchor.x() << " "
              << q_anchor.y() << " " << q_anchor.z() << " " << q_anchor.w()
              << std::endl;

    JitterStats jitter;
    Eigen::Matrix<double, 7, 1> dq_sq_sum = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> c_sq_sum = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Vector3d err_pos_sq_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d err_pos_abs_max = Eigen::Vector3d::Zero();
    double err_pos_inf_max = 0.0;
    Eigen::Vector3d err_ori_sq_sum = Eigen::Vector3d::Zero();
    Eigen::Vector3d err_ori_abs_max = Eigen::Vector3d::Zero();
    double err_ori_norm_max = 0.0;
    std::size_t tick = 0;
    double elapsed = 0.0;

    // 0 none, 1 position tracking error, 2 joint limit, 3 orientation tracking error
    int abort_code = 0;
    int abort_joint = -1;
    double abort_value = 0.0;
    double abort_time = 0.0;

    auto callback = [&](const franka::RobotState& s,
                        franka::Duration period) -> franka::Torques {
      const double dt = period.toSec();
      if (tick > 0) {
        jitter.push(period.toMSec());
      }
      elapsed += dt;

      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(s.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(s.dq.data());

      const PoseData pose = extract_pose_data(model.pose(franka::Frame::kEndEffector, s));
      const Eigen::Vector3d x = pose.position;
      const Eigen::Quaterniond q_cur = pose.quaternion;

      const std::array<double, 42> jac_arr =
          model.zeroJacobian(franka::Frame::kEndEffector, s);
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jac6x7(jac_arr.data());
      const Eigen::Matrix<double, 3, 7> j_p = jac6x7.topRows<3>();
      const Eigen::Matrix<double, 3, 7> j_o = jac6x7.bottomRows<3>();
      const Eigen::Vector3d dx = j_p * dq;
      const Eigen::Vector3d w = j_o * dq;

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

      const double a_eff_o = ori_amp_rad * alpha_amp;
      const double s_omega_o_t = std::sin(omega_o * elapsed);
      const double theta_des = a_eff_o * s_omega_o_t;
      Eigen::Vector3d ori_axis_unit = Eigen::Vector3d::Zero();
      ori_axis_unit(static_cast<int>(args.ori_axis)) = 1.0;
      const Eigen::Quaterniond q_delta(
          Eigen::AngleAxisd(theta_des, ori_axis_unit));
      const Eigen::Quaterniond q_des =
          args.ori_frame_ee ? (q_anchor * q_delta) : (q_delta * q_anchor);

      const Eigen::Vector3d kp_pos_eff = alpha_kp * kp_pos_target;
      const Eigen::Vector3d kp_ori_eff = alpha_kp * kp_ori_target;
      const Eigen::Vector3d e_pos = x_des - x;
      const Eigen::Vector3d e_ori = shortest_quat_error_vec(q_des, q_cur);

      Eigen::Matrix<double, 6, 1> f_task;
      f_task.head<3>() = kp_pos_eff.cwiseProduct(e_pos) - kd_pos.cwiseProduct(dx);
      f_task.tail<3>() = kp_ori_eff.cwiseProduct(e_ori) - kd_ori.cwiseProduct(w);

      Eigen::Matrix<double, 7, 1> c_vec = Eigen::Matrix<double, 7, 1>::Zero();
      if (!args.no_coriolis) {
        const std::array<double, 7> c_arr = model.coriolis(s);
        c_vec = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(c_arr.data());
      }

      const Eigen::Matrix<double, 7, 1> tau_pd =
          jac6x7.transpose() * f_task + c_vec;
      const Eigen::Matrix<double, 7, 1> tau_cmd =
          tau_pd.cwiseMax(-TAU_LIMIT).cwiseMin(TAU_LIMIT);

      dq_sq_sum += dq.cwiseProduct(dq);
      c_sq_sum += c_vec.cwiseProduct(c_vec);
      err_pos_sq_sum += e_pos.cwiseProduct(e_pos);
      err_pos_abs_max = err_pos_abs_max.cwiseMax(e_pos.cwiseAbs());
      const double err_pos_inf = e_pos.cwiseAbs().maxCoeff();
      err_pos_inf_max = std::max(err_pos_inf_max, err_pos_inf);
      err_ori_sq_sum += e_ori.cwiseProduct(e_ori);
      err_ori_abs_max = err_ori_abs_max.cwiseMax(e_ori.cwiseAbs());
      const double err_ori_norm = e_ori.norm();
      err_ori_norm_max = std::max(err_ori_norm_max, err_ori_norm);

      if (args.print_err_every > 0 &&
          tick % static_cast<std::size_t>(args.print_err_every) == 0) {
        Eigen::Index max_idx = 0;
        const double err_inf = e_pos.cwiseAbs().maxCoeff(&max_idx);
        std::cout << "[step5b] t=" << std::fixed << std::setprecision(3) << elapsed
                  << " s"
                  << " |e_pos|_inf=" << std::setprecision(6) << err_inf << " m"
                  << " (" << (max_idx == 0 ? "x" : (max_idx == 1 ? "y" : "z"))
                  << ")"
                  << " ||e_ori||=" << err_ori_norm << " rad" << std::endl;
      }

      if (log_enabled) {
        log_file << std::setprecision(6) << std::fixed << elapsed << ","
                 << period.toMSec();
        for (int j = 0; j < 7; ++j) log_file << "," << s.q[j];
        for (int j = 0; j < 7; ++j) log_file << "," << s.dq[j];
        log_file << "," << x(0) << "," << x(1) << "," << x(2);
        log_file << "," << dx(0) << "," << dx(1) << "," << dx(2);
        log_file << "," << q_cur.x() << "," << q_cur.y() << "," << q_cur.z()
                 << "," << q_cur.w();
        log_file << "," << w(0) << "," << w(1) << "," << w(2);
        log_file << "," << x_des(0) << "," << x_des(1) << "," << x_des(2);
        log_file << "," << dx_des(0) << "," << dx_des(1) << "," << dx_des(2);
        log_file << "," << q_des.x() << "," << q_des.y() << "," << q_des.z()
                 << "," << q_des.w();
        log_file << "," << e_pos(0) << "," << e_pos(1) << "," << e_pos(2);
        log_file << "," << e_ori(0) << "," << e_ori(1) << "," << e_ori(2);
        for (int j = 0; j < 7; ++j) log_file << "," << tau_pd(j);
        for (int j = 0; j < 7; ++j) log_file << "," << tau_cmd(j);
        for (int j = 0; j < 7; ++j) log_file << "," << c_vec(j);
        log_file << "\n";
      }

      ++tick;
      const bool time_up = (args.duration > 0.0 && elapsed >= args.duration);

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
      if (abort_code == 0 && err_pos_inf > CART_TRACK_ABORT_M) {
        abort_code = 1;
        abort_time = elapsed;
        abort_value = err_pos_inf;
      }
      if (abort_code == 0 && err_ori_norm > ORI_TRACK_ABORT_RAD) {
        abort_code = 3;
        abort_time = elapsed;
        abort_value = err_ori_norm;
      }

      if (g_stop_flag.load() || time_up || abort_code != 0) {
        std::array<double, 7> zero{};
        return franka::MotionFinished(franka::Torques(zero));
      }

      std::array<double, 7> tau_out{};
      Eigen::Map<Eigen::Matrix<double, 7, 1>>(tau_out.data()) = tau_cmd;
      return franka::Torques(tau_out);
    };

    std::cout << "[step5b] starting 1 kHz torque control. Ctrl+C to stop."
              << std::endl;
    robot.control(callback);

    record.ticks = tick;
    record.elapsed_s = elapsed;
    record.abort_code = abort_code;
    record.abort_joint = abort_joint;
    record.abort_value = abort_value;
    record.abort_time_s = abort_time;
    record.period_ms_mean = jitter.mean;
    record.period_ms_std = jitter.std();
    record.period_ms_min = jitter.min_v;
    record.period_ms_max = jitter.max_v;

    std::cout << "\n[step5b] --- summary ---\n"
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
      const Eigen::Vector3d err_pos_rms =
          (err_pos_sq_sum / static_cast<double>(tick)).cwiseSqrt();
      const Eigen::Vector3d err_ori_rms =
          (err_ori_sq_sum / static_cast<double>(tick)).cwiseSqrt();
      std::cout << "dq RMS (rad/s) per j       : " << dq_rms.transpose() << "\n"
                << "c  RMS (Nm) per j          : " << c_rms.transpose() << "\n"
                << "pos err RMS (m) xyz        : " << err_pos_rms.transpose()
                << "\n"
                << "pos err |max| (m) xyz      : " << err_pos_abs_max.transpose()
                << "\n"
                << "pos |e|_inf max (m)        : " << err_pos_inf_max << "\n"
                << "ori err RMS (rad) xyz      : " << err_ori_rms.transpose()
                << "\n"
                << "ori err |max| (rad) xyz    : " << err_ori_abs_max.transpose()
                << "\n"
                << "ori ||e|| max (rad)        : " << err_ori_norm_max << "\n";

      record.has_summary = true;
      for (int j = 0; j < 7; ++j) {
        record.dq_rms[j] = dq_rms(j);
        record.c_rms[j] = c_rms(j);
      }
      for (int i = 0; i < 3; ++i) {
        record.err_pos_rms[i] = err_pos_rms(i);
        record.err_pos_abs_max[i] = err_pos_abs_max(i);
        record.err_ori_rms[i] = err_ori_rms(i);
        record.err_ori_abs_max[i] = err_ori_abs_max(i);
      }
      record.err_pos_inf_max = err_pos_inf_max;
      record.err_ori_norm_max = err_ori_norm_max;
    }
    if (abort_code == 1) {
      std::cout << "abort                      : position tracking error at t="
                << abort_time << " s, |e_pos|_inf=" << abort_value
                << " m (threshold " << CART_TRACK_ABORT_M << ")\n";
    } else if (abort_code == 2) {
      std::cout << "abort                      : joint limit at t=" << abort_time
                << " s, joint " << (abort_joint + 1) << ", q=" << abort_value
                << " rad\n";
    } else if (abort_code == 3) {
      std::cout << "abort                      : orientation tracking error at t="
                << abort_time << " s, ||e_ori||=" << abort_value
                << " rad (threshold " << ORI_TRACK_ABORT_RAD << ")\n";
    } else {
      std::cout << "abort                      : none\n";
    }
    if (log_enabled) {
      std::cout << "csv log                    : " << args.log_path << "\n";
    }
    if (!args.sidecar_path.empty()) {
      std::cout << "sidecar json               : " << args.sidecar_path << "\n";
    }
    std::cout << std::flush;
    record.ended_normally = true;
  } catch (const franka::Exception& e) {
    record.had_exception = true;
    record.exception_message = std::string("franka::Exception: ") + e.what();
    std::cerr << "[step5b] " << record.exception_message << std::endl;
    write_sidecar_json(record);
    return 10;
  } catch (const std::exception& e) {
    record.had_exception = true;
    record.exception_message = std::string("std::exception: ") + e.what();
    std::cerr << "[step5b] " << record.exception_message << std::endl;
    write_sidecar_json(record);
    return 11;
  }

  write_sidecar_json(record);
  return 0;
}
