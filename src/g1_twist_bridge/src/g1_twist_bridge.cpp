#include <functional>

#include "g1_twist_bridge/g1_twist_bridge.hpp"

namespace g1_twist_bridge {

G1_twist_bridge::G1_twist_bridge() : Node("g1_twist_bridge") {
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&G1_twist_bridge::CmdVelCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "G1 Twist bridge started, waiting for cmd_vel...");
}

void G1_twist_bridge::CmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg) {
  const double vx = msg->linear.x;
  const double vy = msg->linear.y;
  const double wz = msg->angular.z;

  RCLCPP_INFO(get_logger(), "vx: %.3f m/s, vy: %.3f m/s, wz: %.3f rad/s", vx, vy, wz);
}

}  // namespace g1_twist_bridge
