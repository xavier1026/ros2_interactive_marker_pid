#include <memory>

#include "g1_twist_bridge/g1_twist_bridge.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char const* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<g1_twist_bridge::G1_twist_bridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
