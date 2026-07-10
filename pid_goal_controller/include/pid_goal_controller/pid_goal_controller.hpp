#pragma once

#include <chrono>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pid_goal_controller/pid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace pid_goal_controller {

class PidGoalController : public rclcpp::Node {
 public:
  PidGoalController();

 private:
  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void GoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void PublishDebug(double error_x_body, double error_y_body, double error_yaw, double distance);
  void PublishStop();
  void ResetPid();
  void ControlLoop();

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr error_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  Pid pid_x_;
  Pid pid_y_;
  Pid pid_yaw_;

  bool have_odom_{false};
  bool have_goal_{false};

  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};

  double goal_x_{0.0};
  double goal_y_{0.0};
  double goal_yaw_{0.0};

  double max_v_{0.4};
  double max_w_{1.0};
  double pos_tolerance_{0.03};
  double yaw_tolerance_{0.05};
  double control_frequency_{30.0};
  double odom_timeout_{0.5};

  std::string goal_topic_;
  std::string odom_topic_;
  std::string cmd_vel_topic_;
  std::string error_topic_;
  std::string distance_topic_;

  rclcpp::Time last_time_;
  std::chrono::steady_clock::time_point last_odom_wall_time_;
};

}  // namespace pid_goal_controller
