#include "pid_goal_controller/pid.hpp"

#include <algorithm>
#include <cmath>

namespace pid_goal_controller {
namespace {

double ClampValue(double value, double low, double high) {
  return std::max(low, std::min(value, high));
}

}  // namespace

Pid::Pid() = default;

Pid::Pid(double kp, double ki, double kd, double max_output, double max_integral)
    : kp_(kp),
      ki_(ki),
      kd_(kd),
      max_output_(std::fabs(max_output)),
      max_integral_(std::fabs(max_integral)) {}

void Pid::Reset() {
  integral_ = 0.0;
  last_error_ = 0.0;
  has_last_ = false;
}

double Pid::Update(double error, double dt) {
  if (dt <= 0.0) {
    return 0.0;
  }

  integral_ += error * dt;
  integral_ = ClampValue(integral_, -max_integral_, max_integral_);

  double derivative = 0.0;
  if (has_last_) {
    derivative = (error - last_error_) / dt;
  }

  last_error_ = error;
  has_last_ = true;

  const double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
  return ClampValue(output, -max_output_, max_output_);
}

}  // namespace pid_goal_controller
