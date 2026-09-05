// gripper_cmd.cpp
//
// One-shot Franka Hand command over libfranka's gripper interface.
//
//   gripper_cmd <robot_ip> homing
//   gripper_cmd <robot_ip> move  --width W [--speed S]
//   gripper_cmd <robot_ip> grasp --width W [--speed S] [--force F] [--eps-in E] [--eps-out E]
//   gripper_cmd <robot_ip> stop
//   gripper_cmd <robot_ip> state
//
// The gripper server is its own TCP endpoint on the robot (port 1338), separate
// from the FCI control session (1337) that osc_shm / move_to hold. So this runs
// while the arm controller is up -- the daemon never stops osc_shm for it.
//
// Semantics (libfranka, see franka/gripper.h):
//   move  W S          fingers to width W [m] at S [m/s]; pure position, no force.
//   grasp W S F ei eo  fingers TOWARDS W at S, then squeeze with F [N] once they
//                      stall. W below the object (or negative = past full closure)
//                      means "close as far as you can and hold": the OBJECT sets
//                      the resting width, F sets how hard. "result" is true iff
//                      the final width is within (W - ei, W + eo).
//   homing             calibrate max_width; needed once after power-up or a
//                      finger change.
//   stop               abort the motion in flight (from another connection).
//   state              readOnce() only.
//
// Output: exactly one JSON object on stdout, e.g.
//   {"ok":true,"cmd":"grasp","result":true,"stopped":false,
//    "state":{"width":0.0102,"max_width":0.0800,"is_grasped":true,"temperature":32}}
// "result" is libfranka's return value; "state" is a readOnce() taken after the
// command. Exit 0 whenever the command ran (even if result is false), 1 on a CLI
// error, 10 on franka::Exception.
//
// SIGINT / SIGTERM while move / grasp / homing is in flight -> Gripper::stop() on
// the same connection, then a normal exit with "stopped":true. The daemon's
// gripper_stop signals the running gripper_cmd exactly this way.

#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/gripper_state.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

namespace {

std::atomic<bool> g_stop{false};

void on_signal(int /*signo*/) { g_stop.store(true); }

void usage(const char* argv0) {
  std::cerr
      << "Usage:\n"
      << "  " << argv0 << " <robot_ip> homing\n"
      << "  " << argv0 << " <robot_ip> move  --width W [--speed S]\n"
      << "  " << argv0 << " <robot_ip> grasp --width W [--speed S] [--force F] [--eps-in E] [--eps-out E]\n"
      << "  " << argv0 << " <robot_ip> stop\n"
      << "  " << argv0 << " <robot_ip> state\n"
      << "Units: m, m/s, N. Prints one JSON object on stdout." << std::endl;
}

struct Args {
  std::string cmd;
  double width = 0.08;
  double speed = 0.1;
  double force = 70.0;      // Franka Hand rated continuous grasping force
  double eps_in = 0.005;    // libfranka defaults; the daemon passes robot.yaml's
  double eps_out = 0.005;
  bool has_width = false;
};

bool parse_double(const char* s, double& out) {
  char* end = nullptr;
  out = std::strtod(s, &end);
  return end != s && *end == '\0';
}

bool parse_args(int argc, char** argv, Args& a) {
  a.cmd = argv[2];
  for (int i = 3; i < argc; ++i) {
    const std::string flag = argv[i];
    auto need = [&](double& dst) -> bool {
      if (i + 1 >= argc) return false;
      if (!parse_double(argv[++i], dst)) return false;
      return true;
    };
    if (flag == "--width") {
      if (!need(a.width)) return false;
      a.has_width = true;
    } else if (flag == "--speed") {
      if (!need(a.speed)) return false;
    } else if (flag == "--force") {
      if (!need(a.force)) return false;
    } else if (flag == "--eps-in") {
      if (!need(a.eps_in)) return false;
    } else if (flag == "--eps-out") {
      if (!need(a.eps_out)) return false;
    } else {
      return false;
    }
  }
  if (a.cmd == "move" || a.cmd == "grasp") {
    if (!a.has_width) return false;
    if (a.speed <= 0.0) return false;
  }
  if (a.cmd == "grasp" && a.force <= 0.0) return false;
  return a.cmd == "homing" || a.cmd == "move" || a.cmd == "grasp" || a.cmd == "stop" ||
         a.cmd == "state";
}

std::string json_escape(const std::string& in) {
  std::string out;
  for (char c : in) {
    if (c == '"' || c == '\\') out += '\\';
    if (c == '\n') { out += "\\n"; continue; }
    out += c;
  }
  return out;
}

std::string state_json(const franka::GripperState& s) {
  std::ostringstream o;
  o << std::fixed << std::setprecision(4) << "{\"width\":" << s.width
    << ",\"max_width\":" << s.max_width << ",\"is_grasped\":" << (s.is_grasped ? "true" : "false")
    << ",\"temperature\":" << static_cast<unsigned>(s.temperature) << "}";
  return o.str();
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    usage(argv[0]);
    return 1;
  }
  Args a;
  if (!parse_args(argc, argv, a)) {
    usage(argv[0]);
    return 1;
  }
  const std::string robot_ip = argv[1];

  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  try {
    franka::Gripper gripper(robot_ip);

    bool result = true;
    bool stopped = false;
    if (a.cmd == "state") {
      // nothing to execute
    } else if (a.cmd == "stop") {
      result = gripper.stop();
    } else {
      // Run the blocking command on a worker so the main thread can turn a
      // SIGINT/SIGTERM into Gripper::stop() on the same connection (the pattern
      // deoxys' gripper node uses; the gripper server takes stop concurrently).
      std::atomic<bool> done{false};
      std::exception_ptr worker_error;
      std::thread worker([&]() {
        try {
          if (a.cmd == "homing") {
            result = gripper.homing();
          } else if (a.cmd == "move") {
            result = gripper.move(a.width, a.speed);
          } else {  // grasp
            result = gripper.grasp(a.width, a.speed, a.force, a.eps_in, a.eps_out);
          }
        } catch (...) {
          worker_error = std::current_exception();
        }
        done.store(true);
      });
      while (!done.load()) {
        if (g_stop.load() && !stopped) {
          stopped = true;
          try {
            gripper.stop();
          } catch (const franka::Exception& e) {
            std::cerr << "[gripper_cmd] stop failed: " << e.what() << std::endl;
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      worker.join();
      if (worker_error && !stopped) std::rethrow_exception(worker_error);
      if (stopped) result = false;  // a stopped motion never reached its target
    }

    const franka::GripperState s = gripper.readOnce();
    std::cout << "{\"ok\":true,\"cmd\":\"" << a.cmd << "\",\"result\":" << (result ? "true" : "false")
              << ",\"stopped\":" << (stopped ? "true" : "false") << ",\"state\":" << state_json(s)
              << "}" << std::endl;
  } catch (const franka::Exception& e) {
    std::cerr << "[gripper_cmd] franka::Exception: " << e.what() << std::endl;
    std::cout << "{\"ok\":false,\"cmd\":\"" << a.cmd << "\",\"error\":\"" << json_escape(e.what())
              << "\"}" << std::endl;
    return 10;
  }
  return 0;
}
