// read_current_q.cpp
//
// Read one RobotState from libfranka and print current joint positions.
// This utility does not start a control loop and does not command any motion.

#include <franka/exception.h>
#include <franka/robot.h>

#include <array>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>

namespace {

void print_usage(const char* prog) {
  std::cerr << "Usage: " << prog << " <robot_ip>" << std::endl;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    print_usage(argv[0]);
    return 1;
  }

  const std::string robot_ip = argv[1];
  if (robot_ip.empty() || robot_ip[0] == '-') {
    print_usage(argv[0]);
    return 1;
  }

  try {
    franka::Robot robot(robot_ip);
    franka::RobotState state = robot.readOnce();

    std::array<double, 7> q{};
    for (std::size_t i = 0; i < q.size(); ++i) {
      q[i] = state.q[i];
    }

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "[read_current_q] robot_ip=" << robot_ip << "\n";
    std::cout << "[read_current_q] q (rad): ";
    for (std::size_t i = 0; i < q.size(); ++i) {
      std::cout << q[i];
      if (i + 1 < q.size()) std::cout << " ";
    }
    std::cout << "\n";

    std::cout << "[read_current_q] for step1 --q-des: ";
    for (std::size_t i = 0; i < q.size(); ++i) {
      std::cout << q[i];
      if (i + 1 < q.size()) std::cout << " ";
    }
    std::cout << std::endl;
  } catch (const franka::Exception& e) {
    std::cerr << "[read_current_q] franka::Exception: " << e.what() << std::endl;
    return 10;
  } catch (const std::exception& e) {
    std::cerr << "[read_current_q] std::exception: " << e.what() << std::endl;
    return 11;
  }

  return 0;
}
