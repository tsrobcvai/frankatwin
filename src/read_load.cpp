// read_load.cpp
//
// One-shot diagnostic: connect to the robot, read the configured end-effector /
// load dynamic parameters once, print them, and exit.  These are CONFIGURED
// values (Desk end-effector + any prior setLoad), reported directly by libfranka
// in the RobotState -- they are NOT inferred from joint torque.
//
//   m_ee     : end-effector mass (the Desk-configured EE, e.g. a Franka Hand)
//   m_load   : external load mass (whatever setLoad set; 0 if never called)
//   m_total  : m_ee + m_load  (what the controller actually gravity-compensates)
//   F_x_Cee  : flange->EE  center of mass [m]
//   F_x_Cload: flange->load center of mass [m]
//
// Use this to decide what to put in config robot.yaml `load.mass`:
//   * If m_ee already includes a gripper, load.mass = camera+bracket ONLY.
//   * If m_ee is ~0 (bare flange), load.mass = the whole mounted assembly.
//
// NOTE: run this with the daemon STOPPED -- libfranka grants only one FCI
// session at a time, so osc_shm must not be holding the connection.

#include <franka/exception.h>
#include <franka/robot.h>

#include <array>
#include <iomanip>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <robot_ip>\n"
              << "  (run with the daemon stopped -- needs exclusive FCI access)"
              << std::endl;
    return 1;
  }
  const std::string robot_ip = argv[1];
  try {
    franka::Robot robot(robot_ip);
    franka::RobotState s = robot.readOnce();
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "[read_load] m_ee    = " << s.m_ee << " kg\n"
              << "[read_load] m_load  = " << s.m_load << " kg\n"
              << "[read_load] m_total = " << s.m_total << " kg "
              << "(arm gravity-compensates this on top of the links)\n"
              << "[read_load] F_x_Cee    = [" << s.F_x_Cee[0] << ", "
              << s.F_x_Cee[1] << ", " << s.F_x_Cee[2] << "] m\n"
              << "[read_load] F_x_Cload  = [" << s.F_x_Cload[0] << ", "
              << s.F_x_Cload[1] << ", " << s.F_x_Cload[2] << "] m"
              << std::endl;
  } catch (const franka::Exception& e) {
    std::cerr << "[read_load] franka::Exception: " << e.what() << std::endl;
    return 1;
  }
  return 0;
}
