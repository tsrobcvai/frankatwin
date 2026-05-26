// step5c_excite.cpp
//
// Jacobian-transpose Cartesian PD replay from an externally generated trajectory
// CSV at 1 kHz. Control law and logging schema are aligned with step5b_cart_pose.
//
// Control law:
//   tau_cmd = J^T * F_task + c(q, dq)
//   F_task  = [ Kp_pos * (x_des - x) - Kd_pos * v ;
//               Kp_ori * e_o         - Kd_ori * w ]
//
// Trajectory source:
//   --traj-csv with columns:
//   t_s,x_des_x,x_des_y,x_des_z,dx_des_x,dx_des_y,dx_des_z,
//   quat_des_x,quat_des_y,quat_des_z,quat_des_w

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
#include <vector>

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

constexpr double CART_TRACK_ABORT_M_DEFAULT = 0.05;
constexpr double ORI_TRACK_ABORT_RAD_DEFAULT = 0.30;

std::atomic<bool> g_stop_flag{false};
void signal_handler(int /*signo*/) { g_stop_flag.store(true); }

struct Args {
  std::string robot_ip;
  std::string traj_csv;
  double kp_pos{200.0};
  bool kd_pos_set{false};
  double kd_pos{0.0};
  double kp_ori{20.0};
  bool kd_ori_set{false};
  double kd_ori{0.0};
  double duration{-1.0};  // < 0 means derive from trajectory t_s
  double ramp{1.5};
  bool no_coriolis{false};
  int print_err_every{100};
  // Runtime safety abort thresholds. Defaults match step5b/step5c (small-amp
  // sweeps). step5d uses larger Cartesian + rotation amplitudes and should
  // pass --cart-abort / --ori-abort with looser thresholds because this
  // controller has no velocity feedforward and tracking lag scales with amp.
  double cart_track_abort_m{CART_TRACK_ABORT_M_DEFAULT};
  double ori_track_abort_rad{ORI_TRACK_ABORT_RAD_DEFAULT};
  std::string log_path;
  std::string sidecar_path;
};

struct TrajSample {
  double t_s{0.0};
  Eigen::Vector3d x_des{Eigen::Vector3d::Zero()};
  Eigen::Vector3d dx_des{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q_des{Eigen::Quaterniond::Identity()};
};

struct PoseData {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
  Eigen::Quaterniond quaternion{Eigen::Quaterniond::Identity()};
};

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

bool parse_args(int argc, char** argv, Args& out) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <robot_ip> --traj-csv path "
              << "[--kp-pos K] [--kd-pos K] [--kp-ori K] [--kd-ori K] "
              << "[--duration sec] [--ramp sec] [--no-coriolis] "
              << "[--cart-abort m] [--ori-abort rad] "
              << "[--print-err-every N] [--log path] [--sidecar path]\n";
    return false;
  }
  out.robot_ip = argv[1];
  if (out.robot_ip.empty() || out.robot_ip[0] == '-') return false;

  int i = 2;
  while (i < argc) {
    const std::string key = argv[i];
    if (key == "--traj-csv") {
      if (i + 1 >= argc) return false;
      out.traj_csv = argv[i + 1];
      i += 2;
    } else if (key == "--kp-pos") {
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
    } else if (key == "--duration") {
      if (i + 1 >= argc) return false;
      out.duration = std::atof(argv[i + 1]);
      i += 2;
    } else if (key == "--ramp") {
      if (i + 1 >= argc) return false;
      out.ramp = std::atof(argv[i + 1]);
      i += 2;
    } else if (key == "--no-coriolis") {
      out.no_coriolis = true;
      i += 1;
    } else if (key == "--cart-abort") {
      if (i + 1 >= argc) return false;
      out.cart_track_abort_m = std::atof(argv[i + 1]);
      i += 2;
    } else if (key == "--ori-abort") {
      if (i + 1 >= argc) return false;
      out.ori_track_abort_rad = std::atof(argv[i + 1]);
      i += 2;
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
      return false;
    } else {
      std::cerr << "[step5c] unknown arg: " << key << std::endl;
      return false;
    }
  }
  if (out.traj_csv.empty()) {
    std::cerr << "[step5c] --traj-csv is required." << std::endl;
    return false;
  }
  if (!out.kd_pos_set) out.kd_pos = 2.0 * std::sqrt(out.kp_pos);
  if (!out.kd_ori_set) out.kd_ori = 2.0 * std::sqrt(out.kp_ori);
  if (out.ramp < 0.0) out.ramp = 0.0;
  if (out.print_err_every < 0) return false;
  if (!(out.cart_track_abort_m > 0.0)) {
    std::cerr << "[step5c] --cart-abort must be > 0 (got "
              << out.cart_track_abort_m << ")\n";
    return false;
  }
  if (!(out.ori_track_abort_rad > 0.0)) {
    std::cerr << "[step5c] --ori-abort must be > 0 (got "
              << out.ori_track_abort_rad << ")\n";
    return false;
  }
  return true;
}

std::vector<std::string> split_csv(const std::string& line) {
  std::vector<std::string> out;
  std::string cur;
  for (char c : line) {
    if (c == ',') {
      out.push_back(cur);
      cur.clear();
    } else {
      cur.push_back(c);
    }
  }
  out.push_back(cur);
  return out;
}

int index_of(const std::vector<std::string>& header, const std::string& key) {
  for (std::size_t i = 0; i < header.size(); ++i) {
    if (header[i] == key) return static_cast<int>(i);
  }
  return -1;
}

std::vector<TrajSample> load_traj_csv(const std::string& path) {
  std::ifstream f(path);
  if (!f.is_open()) {
    throw std::runtime_error("cannot open traj csv: " + path);
  }

  std::string header_line;
  if (!std::getline(f, header_line)) {
    throw std::runtime_error("empty traj csv: " + path);
  }
  auto h = split_csv(header_line);
  const int i_t = index_of(h, "t_s");
  const int i_x = index_of(h, "x_des_x");
  const int i_y = index_of(h, "x_des_y");
  const int i_z = index_of(h, "x_des_z");
  const int i_dx = index_of(h, "dx_des_x");
  const int i_dy = index_of(h, "dx_des_y");
  const int i_dz = index_of(h, "dx_des_z");
  const int i_qx = index_of(h, "quat_des_x");
  const int i_qy = index_of(h, "quat_des_y");
  const int i_qz = index_of(h, "quat_des_z");
  const int i_qw = index_of(h, "quat_des_w");
  if (i_t < 0 || i_x < 0 || i_y < 0 || i_z < 0 || i_dx < 0 || i_dy < 0 ||
      i_dz < 0 || i_qx < 0 || i_qy < 0 || i_qz < 0 || i_qw < 0) {
    throw std::runtime_error(
        "traj csv missing required columns: t_s/x_des/dx_des/quat_des");
  }

  std::vector<TrajSample> traj;
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty()) continue;
    auto cols = split_csv(line);
    const int need = static_cast<int>(h.size());
    if (static_cast<int>(cols.size()) < need) continue;
    TrajSample s;
    s.t_s = std::stod(cols[i_t]);
    s.x_des = Eigen::Vector3d(std::stod(cols[i_x]), std::stod(cols[i_y]),
                              std::stod(cols[i_z]));
    s.dx_des = Eigen::Vector3d(std::stod(cols[i_dx]), std::stod(cols[i_dy]),
                               std::stod(cols[i_dz]));
    s.q_des = Eigen::Quaterniond(std::stod(cols[i_qw]), std::stod(cols[i_qx]),
                                 std::stod(cols[i_qy]), std::stod(cols[i_qz]));
    s.q_des.normalize();
    traj.push_back(s);
  }
  if (traj.empty()) {
    throw std::runtime_error("traj csv has no data rows: " + path);
  }
  return traj;
}

bool try_set_realtime_priority(int priority = 80) {
  sched_param sp{};
  sp.sched_priority = priority;
  int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
  if (rc != 0) {
    std::cerr << "[step5c] WARN: failed to set SCHED_FIFO (rc=" << rc
              << ", errno=" << std::strerror(errno)
              << "). Continuing without RT priority." << std::endl;
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
  std::string controller{"step5c_excite"};
  std::string started_utc;
  std::string csv_path;
  std::string sidecar_path;
  std::string robot_ip;
  std::string traj_csv;
  int control_rate_hz{1000};
  std::string frame{"panda_link0 (base)"};
  bool rt_priority{false};
  Args args;
  bool has_init{false};
  std::array<double, 7> q_init{};
  std::array<double, 3> x_anchor{};
  std::array<double, 4> q_anchor_xyzw{};
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
    std::cerr << "[step5c] WARN: cannot open sidecar JSON for write: "
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
  f << "  \"traj_csv\": \"" << json_escape(r.traj_csv) << "\",\n";
  f << "  \"control_rate_hz\": " << r.control_rate_hz << ",\n";
  f << "  \"frame\": \"" << json_escape(r.frame) << "\",\n";
  f << "  \"rt_priority\": " << (r.rt_priority ? "true" : "false") << ",\n";
  f << "  \"args\": {\n";
  f << "    \"kp_pos\": " << json_double(r.args.kp_pos) << ",\n";
  f << "    \"kd_pos\": " << json_double(r.args.kd_pos) << ",\n";
  f << "    \"kp_ori\": " << json_double(r.args.kp_ori) << ",\n";
  f << "    \"kd_ori\": " << json_double(r.args.kd_ori) << ",\n";
  f << "    \"duration\": " << json_double(r.args.duration) << ",\n";
  f << "    \"ramp\": " << json_double(r.args.ramp) << ",\n";
  f << "    \"no_coriolis\": " << (r.args.no_coriolis ? "true" : "false")
    << ",\n";
  f << "    \"print_err_every\": " << r.args.print_err_every << ",\n";
  f << "    \"cart_track_abort_m\": " << json_double(r.args.cart_track_abort_m)
    << ",\n";
  f << "    \"ori_track_abort_rad\": "
    << json_double(r.args.ori_track_abort_rad) << "\n";
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
    f << "    \"err_ori_norm_max\": " << json_double(r.err_ori_norm_max) << "\n";
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
  record.traj_csv = args.traj_csv;
  record.args = args;

  std::vector<TrajSample> traj;
  try {
    traj = load_traj_csv(args.traj_csv);
  } catch (const std::exception& e) {
    std::cerr << "[step5c] failed to load trajectory: " << e.what() << std::endl;
    return 2;
  }
  if (args.duration < 0.0) args.duration = traj.back().t_s;
  record.args.duration = args.duration;

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
  const bool rt_ok = try_set_realtime_priority();
  record.rt_priority = rt_ok;

  const Eigen::Vector3d kp_pos_target = Eigen::Vector3d::Constant(args.kp_pos);
  const Eigen::Vector3d kd_pos = Eigen::Vector3d::Constant(args.kd_pos);
  const Eigen::Vector3d kp_ori_target = Eigen::Vector3d::Constant(args.kp_ori);
  const Eigen::Vector3d kd_ori = Eigen::Vector3d::Constant(args.kd_ori);

  std::cout << "[step5c] robot_ip = " << args.robot_ip << "\n"
            << "[step5c] traj_csv = " << args.traj_csv << "\n"
            << "[step5c] samples  = " << traj.size() << "\n"
            << "[step5c] duration = " << args.duration << " s\n"
            << "[step5c] kp_pos/kd_pos = " << args.kp_pos << " / " << args.kd_pos
            << "\n"
            << "[step5c] kp_ori/kd_ori = " << args.kp_ori << " / " << args.kd_ori
            << "\n"
            << "[step5c] ramp = " << args.ramp << " s\n"
            << "[step5c] coriolis = "
            << (args.no_coriolis ? "disabled (--no-coriolis)" : "enabled")
            << "\n"
            << "[step5c] abort thresholds: |e_pos|_inf > "
            << args.cart_track_abort_m << " m, ||e_ori|| > "
            << args.ori_track_abort_rad << " rad"
            << std::endl;

  std::ofstream log_file;
  bool log_enabled = false;
  if (!args.log_path.empty()) {
    log_file.open(args.log_path);
    if (!log_file.is_open()) {
      std::cerr << "[step5c] cannot open log file: " << args.log_path << std::endl;
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
      std::cerr << "[step5c] q_init exceeds nominal Panda joint limits."
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
    int abort_code = 0;
    int abort_joint = -1;
    double abort_value = 0.0;
    double abort_time = 0.0;

    auto callback = [&](const franka::RobotState& s,
                        franka::Duration period) -> franka::Torques {
      const double dt = period.toSec();
      if (tick > 0) jitter.push(period.toMSec());
      elapsed += dt;
      const std::size_t idx = std::min<std::size_t>(tick, traj.size() - 1);
      const TrajSample& trg = traj[idx];

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
      const Eigen::Vector3d kp_pos_eff = alpha_kp * kp_pos_target;
      const Eigen::Vector3d kp_ori_eff = alpha_kp * kp_ori_target;

      const Eigen::Vector3d e_pos = trg.x_des - x;
      const Eigen::Vector3d e_ori = shortest_quat_error_vec(trg.q_des, q_cur);
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
        std::cout << "[step5c] t=" << std::fixed << std::setprecision(3)
                  << elapsed << " s |e_pos|_inf=" << std::setprecision(6)
                  << err_pos_inf << " m ||e_ori||=" << err_ori_norm << " rad"
                  << std::endl;
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
        log_file << "," << trg.x_des(0) << "," << trg.x_des(1) << ","
                 << trg.x_des(2);
        log_file << "," << trg.dx_des(0) << "," << trg.dx_des(1) << ","
                 << trg.dx_des(2);
        log_file << "," << trg.q_des.x() << "," << trg.q_des.y() << ","
                 << trg.q_des.z() << "," << trg.q_des.w();
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
      if (abort_code == 0 && err_pos_inf > args.cart_track_abort_m) {
        abort_code = 1;
        abort_time = elapsed;
        abort_value = err_pos_inf;
      }
      if (abort_code == 0 && err_ori_norm > args.ori_track_abort_rad) {
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

    if (tick > 0) {
      const Eigen::Matrix<double, 7, 1> dq_rms =
          (dq_sq_sum / static_cast<double>(tick)).cwiseSqrt();
      const Eigen::Matrix<double, 7, 1> c_rms =
          (c_sq_sum / static_cast<double>(tick)).cwiseSqrt();
      const Eigen::Vector3d err_pos_rms =
          (err_pos_sq_sum / static_cast<double>(tick)).cwiseSqrt();
      const Eigen::Vector3d err_ori_rms =
          (err_ori_sq_sum / static_cast<double>(tick)).cwiseSqrt();
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
    record.ended_normally = true;
  } catch (const franka::Exception& e) {
    record.had_exception = true;
    record.exception_message = std::string("franka::Exception: ") + e.what();
    std::cerr << "[step5c] " << record.exception_message << std::endl;
    write_sidecar_json(record);
    return 10;
  } catch (const std::exception& e) {
    record.had_exception = true;
    record.exception_message = std::string("std::exception: ") + e.what();
    std::cerr << "[step5c] " << record.exception_message << std::endl;
    write_sidecar_json(record);
    return 11;
  }

  write_sidecar_json(record);
  return 0;
}
