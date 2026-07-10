#pragma once

namespace pid_goal_controller {

class Pid {
 public:
  Pid();
  Pid(double kp, double ki, double kd, double max_output, double max_integral);

  void Reset();
  double Update(double error, double dt);

 private:
  double kp_{0.0};
  double ki_{0.0};
  double kd_{0.0};

  double max_output_{0.0};
  double max_integral_{0.0};

  double integral_{0.0};
  double last_error_{0.0};
  bool has_last_{false};
};

}  // namespace pid_goal_controller
