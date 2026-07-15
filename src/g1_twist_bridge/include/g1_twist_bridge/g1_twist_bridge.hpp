#pragma once

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace g1_twist_bridge {

class G1_twist_bridge : public rclcpp::Node {
 public:
  G1_twist_bridge();

 private:
  void CmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg);
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

}  // namespace g1_twist_bridge
