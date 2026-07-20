#include <memory>

#include "g1_twist_bridge/g1_twist_bridge.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char const* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<g1_twist_bridge::G1TwistBridge>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
