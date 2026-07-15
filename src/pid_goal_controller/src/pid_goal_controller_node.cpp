#include <memory>

#include "pid_goal_controller/pid_goal_controller.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pid_goal_controller::PidGoalController>());
  rclcpp::shutdown();
  return 0;
}
