#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "pid_goal_controller/pid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace pid_goal_controller {

class PidGoalController : public rclcpp::Node {
 public:
  PidGoalController();
  ~PidGoalController() noexcept override;

 private:
  void GoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void PublishDebug(double error_x_body, double error_y_body, double error_yaw, double distance);
  void PublishStop();
  void RejectGoal();
  void ResetPid();
  void ControlLoop();

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr error_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_pub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::TimerBase::SharedPtr timer_;

  Pid pid_x_;
  Pid pid_y_;
  Pid pid_yaw_;

  std::mutex mtx_;

  bool have_goal_{false};
  bool publish_debug_{true};

  geometry_msgs::msg::PoseStamped goal_pose_;
  std::uint64_t goal_generation_{0};

  double max_v_{0.10};
  double max_w_{0.15};
  double pos_tolerance_{0.10};
  double yaw_tolerance_{0.10};
  double control_frequency_{30.0};
  double transform_timeout_{0.1};

  std::string goal_topic_;
  std::string global_frame_;
  std::string base_frame_;
  std::string cmd_vel_topic_;
  std::string error_topic_;
  std::string distance_topic_;

  std::chrono::steady_clock::time_point last_time_;
};

}  // namespace pid_goal_controller
