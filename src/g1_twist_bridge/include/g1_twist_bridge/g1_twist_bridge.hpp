#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"

namespace g1_twist_bridge {

class G1TwistBridge : public rclcpp::Node {
 public:
  G1TwistBridge();
  ~G1TwistBridge() noexcept override;

 private:
  using SteadyClock = std::chrono::steady_clock;

  void CmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg);
  void ResponseCallback(const unitree_api::msg::Response::ConstSharedPtr msg);
  void SendTimerCallback();

  rcl_interfaces::msg::SetParametersResult OnSetParameters(
      const std::vector<rclcpp::Parameter>& parameters);

  bool PublishVelocityLocked(double vx, double vy, double wz) noexcept;
  void PublishStopLocked(const char* reason) noexcept;
  void ClearCommandLocked() noexcept;

  std::string cmd_vel_topic_;
  std::string request_topic_;
  std::string response_topic_;

  std::mutex state_mutex_;
  bool armed_{false};
  bool moving_{false};
  bool received_cmd_{false};
  double target_vx_{0.0};
  double target_vy_{0.0};
  double target_wz_{0.0};
  double max_vx_{0.10};
  double max_vy_{0.05};
  double max_wz_{0.15};
  double deadband_{0.001};
  double send_frequency_{10.0};
  double command_duration_{0.20};
  double command_timeout_{0.40};
  SteadyClock::time_point last_cmd_time_;

  std::atomic<int64_t> next_request_id_;

  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr request_pub_;
  rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr response_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr send_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

}  // namespace g1_twist_bridge
