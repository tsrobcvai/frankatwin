// read_current_pose.cpp
//
// One-shot utility that calls libfranka's `robot.readOnce()`, extracts the
// end-effector pose via `model.pose(kEndEffector, state)`, and writes a JSON
// document in the step5b sidecar schema so it can be consumed directly by
// `panda_control/scripts/gen_excitation_traj.py --base-sidecar <path>`.
//
// Schema (only the keys gen_excitation_traj.py reads are emitted; full
// step5b-style fields are included verbatim so downstream tooling that
// validates the schema still passes):
//   {
//     "schema_version": 1,
//     "controller": "read_current_pose",
//     "started_utc": "...",
//     "robot_ip": "...",
//     "q_init": [7],
//     "x_anchor": [3],
//     "q_anchor_xyzw": [4],
//     "args": {
//       "kp_pos": 200.0, "kd_pos": 28.284271247461902,
//       "kp_ori":  20.0, "kd_ori":  8.94427190999916,
//       "ramp": 1.5, "amp_ramp": 1.5,
//       "no_coriolis": false
//     }
//   }
//
// CLI:
//   ./read_current_pose <robot_ip> [--out path/to/anchor.json]
//                                  [--kp-pos K] [--kp-ori K] [--ramp sec]
//
// The output JSON is always also printed to stdout (so it can be piped if
// --out is not provided).

#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace {

struct Args {
  std::string robot_ip;
  std::string out_path;
  double kp_pos{200.0};
  double kp_ori{20.0};
  double ramp{1.5};
};

void print_usage(const char* prog) {
  std::cerr << "Usage: " << prog << " <robot_ip>"
            << " [--out path/to/anchor.json]"
            << " [--kp-pos K] [--kp-ori K] [--ramp sec]"
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
    if (key == "--out") {
      if (i + 1 >= argc) return false;
      out.out_path = argv[i + 1];
      i += 2;
    } else if (key == "--kp-pos") {
      if (i + 1 >= argc) return false;
      out.kp_pos = std::atof(argv[i + 1]);
      i += 2;
    } else if (key == "--kp-ori") {
      if (i + 1 >= argc) return false;
      out.kp_ori = std::atof(argv[i + 1]);
      i += 2;
    } else if (key == "--ramp") {
      if (i + 1 >= argc) return false;
      out.ramp = std::atof(argv[i + 1]);
      i += 2;
    } else if (key == "-h" || key == "--help") {
      print_usage(argv[0]);
      return false;
    } else {
      std::cerr << "[read_current_pose] unknown arg: " << key << std::endl;
      print_usage(argv[0]);
      return false;
    }
  }
  return true;
}

struct PoseData {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
  Eigen::Quaterniond quaternion{Eigen::Quaterniond::Identity()};
};

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

std::string utc_iso8601_now() {
  const std::time_t t = std::time(nullptr);
  std::tm gm{};
  gmtime_r(&t, &gm);
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

std::string build_payload(const Args& args,
                          const std::array<double, 7>& q,
                          const Eigen::Vector3d& x,
                          const Eigen::Quaterniond& quat) {
  const double kd_pos = 2.0 * std::sqrt(args.kp_pos);
  const double kd_ori = 2.0 * std::sqrt(args.kp_ori);

  std::array<double, 3> x_arr = {x.x(), x.y(), x.z()};
  std::array<double, 4> q_arr = {quat.x(), quat.y(), quat.z(), quat.w()};

  std::ostringstream f;
  f << "{\n";
  f << "  \"schema_version\": 1,\n";
  f << "  \"controller\": \"read_current_pose\",\n";
  f << "  \"started_utc\": \"" << json_escape(utc_iso8601_now()) << "\",\n";
  f << "  \"robot_ip\": \"" << json_escape(args.robot_ip) << "\",\n";
  f << "  \"frame\": \"panda_link0 (base)\",\n";
  f << "  \"q_init\": " << json_double_array(q.data(), 7) << ",\n";
  f << "  \"x_anchor\": " << json_double_array(x_arr.data(), 3) << ",\n";
  f << "  \"q_anchor_xyzw\": " << json_double_array(q_arr.data(), 4) << ",\n";
  f << "  \"args\": {\n";
  f << "    \"kp_pos\": " << json_double(args.kp_pos) << ",\n";
  f << "    \"kd_pos\": " << json_double(kd_pos) << ",\n";
  f << "    \"kp_ori\": " << json_double(args.kp_ori) << ",\n";
  f << "    \"kd_ori\": " << json_double(kd_ori) << ",\n";
  f << "    \"ramp\": " << json_double(args.ramp) << ",\n";
  f << "    \"amp_ramp\": " << json_double(args.ramp) << ",\n";
  f << "    \"no_coriolis\": false\n";
  f << "  }\n";
  f << "}\n";
  return f.str();
}

}  // namespace

int main(int argc, char** argv) {
  Args args;
  if (!parse_args(argc, argv, args)) return 1;

  try {
    franka::Robot robot(args.robot_ip);
    franka::Model model = robot.loadModel();
    franka::RobotState state = robot.readOnce();

    std::array<double, 7> q{};
    for (int j = 0; j < 7; ++j) q[j] = state.q[j];

    const PoseData pose =
        extract_pose_data(model.pose(franka::Frame::kEndEffector, state));
    const Eigen::Vector3d x = pose.position;
    const Eigen::Quaterniond quat = pose.quaternion;

    const std::string payload = build_payload(args, q, x, quat);

    std::cerr << "[read_current_pose] robot_ip = " << args.robot_ip << "\n"
              << "[read_current_pose] q (rad)  = ";
    std::cerr << std::fixed << std::setprecision(6);
    for (int j = 0; j < 7; ++j) {
      std::cerr << q[j];
      if (j + 1 < 7) std::cerr << " ";
    }
    std::cerr << "\n[read_current_pose] x (m)    = " << x.x() << " " << x.y()
              << " " << x.z() << "\n"
              << "[read_current_pose] quat xyzw = " << quat.x() << " "
              << quat.y() << " " << quat.z() << " " << quat.w() << std::endl;

    if (!args.out_path.empty()) {
      std::ofstream f(args.out_path);
      if (!f.is_open()) {
        std::cerr << "[read_current_pose] cannot open --out path for write: "
                  << args.out_path << std::endl;
        return 3;
      }
      f << payload;
      f.close();
      std::cerr << "[read_current_pose] wrote " << args.out_path << std::endl;
    }

    std::cout << payload;
  } catch (const franka::Exception& e) {
    std::cerr << "[read_current_pose] franka::Exception: " << e.what()
              << std::endl;
    return 10;
  } catch (const std::exception& e) {
    std::cerr << "[read_current_pose] std::exception: " << e.what()
              << std::endl;
    return 11;
  }
  return 0;
}
