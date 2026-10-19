#pragma once

#include <algorithm>

namespace frankatwin {

inline double joint_pd(double q, double dq, double target_q, double target_dq,
                       double kp, double kd, double coriolis) {
  return kp * (target_q - q) + kd * (target_dq - dq) + coriolis;
}

inline double limit_effort(double effort, double previous, double magnitude,
                           double rate, double dt) {
  effort = std::clamp(effort, -magnitude, magnitude);
  if (rate > 0.0) {
    const double delta = rate * (dt > 0.0 ? dt : 1e-3);
    effort = std::clamp(effort, previous - delta, previous + delta);
  }
  return effort;
}

}  // namespace frankatwin
