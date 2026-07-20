#include "g1_twist_bridge/g1_twist_bridge.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <functional>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace g1_twist_bridge {
namespace {

constexpr int64_t kSetVelocityApiId = 7105;
constexpr auto kLogThrottleDuration = 2000;

int64_t InitialRequestId() {
  return static_cast<int64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                  std::chrono::steady_clock::now().time_since_epoch())
                                  .count());
}

double ApplyDeadband(double value, double deadband) {
  if (value == 0.0 || std::fabs(value) < deadband) {
    return 0.0;
  }
  return value;
}

bool IsZeroVelocity(double vx, double vy, double wz) { return vx == 0.0 && vy == 0.0 && wz == 0.0; }

std::string VelocityJson(double vx, double vy, double wz, double duration) {
  std::ostringstream stream;
  stream.imbue(std::locale::classic());
  stream << std::fixed << std::setprecision(6) << "{\"velocity\":[" << vx << ',' << vy << ',' << wz
         << "],\"duration\":" << duration << '}';
  return stream.str();
}

std::string ValidateConfiguration(const std::string& cmd_vel_topic,
                                  const std::string& request_topic,
                                  const std::string& response_topic, double max_vx, double max_vy,
                                  double max_wz, double deadband, double send_frequency,
                                  double command_duration, double command_timeout) {
  if (cmd_vel_topic.empty() || request_topic.empty() || response_topic.empty()) {
    return "cmd_vel_topic, request_topic, and response_topic must not be empty";
  }
  if (!std::isfinite(max_vx) || max_vx <= 0.0 || !std::isfinite(max_vy) || max_vy <= 0.0 ||
      !std::isfinite(max_wz) || max_wz <= 0.0) {
    return "max_vx, max_vy, and max_wz must be finite and greater than zero";
  }
  if (!std::isfinite(deadband) || deadband < 0.0) {
    return "deadband must be finite and greater than or equal to zero";
  }
  if (!std::isfinite(send_frequency) || send_frequency < 1.0 || send_frequency > 50.0) {
    return "send_frequency must be finite and within [1.0, 50.0] Hz";
  }
  if (!std::isfinite(command_duration) || command_duration < 1.0 / send_frequency ||
      command_duration > 2.0) {
    return "command_duration must be finite, at least one send period, and at most 2.0 s";
  }
  if (!std::isfinite(command_timeout) || command_timeout < command_duration) {
    return "command_timeout must be finite and greater than or equal to command_duration";
  }
  return {};
}

rcl_interfaces::msg::SetParametersResult FailedParameterResult(std::string reason) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason = std::move(reason);
  return result;
}

}  // namespace

G1TwistBridge::G1TwistBridge()
    : Node("g1_twist_bridge"),
      last_cmd_time_(SteadyClock::now()),
      next_request_id_(InitialRequestId()) {
  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  request_topic_ = declare_parameter<std::string>("request_topic", "/api/sport/request");
  response_topic_ = declare_parameter<std::string>("response_topic", "/api/sport/response");

  armed_ = declare_parameter<bool>("armed", false);
  max_vx_ = declare_parameter<double>("max_vx", 0.10);
  max_vy_ = declare_parameter<double>("max_vy", 0.05);
  max_wz_ = declare_parameter<double>("max_wz", 0.15);
  deadband_ = declare_parameter<double>("deadband", 0.001);
  send_frequency_ = declare_parameter<double>("send_frequency", 10.0);
  command_duration_ = declare_parameter<double>("command_duration", 0.20);
  command_timeout_ = declare_parameter<double>("command_timeout", 0.40);

  const std::string validation_error = ValidateConfiguration(
      cmd_vel_topic_, request_topic_, response_topic_, max_vx_, max_vy_, max_wz_, deadband_,
      send_frequency_, command_duration_, command_timeout_);
  if (!validation_error.empty()) {
    throw std::invalid_argument("Invalid g1_twist_bridge parameters: " + validation_error);
  }

  request_pub_ = create_publisher<unitree_api::msg::Request>(request_topic_, 10);
  response_sub_ = create_subscription<unitree_api::msg::Response>(
      response_topic_, 10,
      std::bind(&G1TwistBridge::ResponseCallback, this, std::placeholders::_1));
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 10, std::bind(&G1TwistBridge::CmdVelCallback, this, std::placeholders::_1));

  parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&G1TwistBridge::OnSetParameters, this, std::placeholders::_1));

  const auto send_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / send_frequency_));
  send_timer_ = create_wall_timer(send_period, std::bind(&G1TwistBridge::SendTimerCallback, this));

  RCLCPP_INFO(get_logger(), "Native Unitree bridge ready: %s -> %s (SetVelocity API %lld, %.1f Hz)",
              cmd_vel_topic_.c_str(), request_topic_.c_str(),
              static_cast<long long>(kSetVelocityApiId), send_frequency_);
  RCLCPP_INFO(get_logger(), "Velocity limits: vx=%.3f m/s, vy=%.3f m/s, wz=%.3f rad/s", max_vx_,
              max_vy_, max_wz_);
  if (!armed_) {
    RCLCPP_WARN(get_logger(), "Bridge is DISARMED; non-zero velocity commands are blocked.");
  }
}

G1TwistBridge::~G1TwistBridge() noexcept {
  try {
    if (send_timer_) {
      send_timer_->cancel();
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    ClearCommandLocked();
    PublishStopLocked("node shutdown");
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(get_logger(), "Failed to publish shutdown stop: %s", exception.what());
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "Failed to publish shutdown stop: unknown exception");
  }
}

void G1TwistBridge::CmdVelCallback(const geometry_msgs::msg::Twist::ConstSharedPtr msg) {
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y) ||
      !std::isfinite(msg->angular.z)) {
    ClearCommandLocked();
    PublishStopLocked("invalid cmd_vel");
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), kLogThrottleDuration,
                          "Rejected cmd_vel containing NaN or infinity; sent zero velocity.");
    return;
  }

  const double vx = ApplyDeadband(std::clamp(msg->linear.x, -max_vx_, max_vx_), deadband_);
  const double vy = ApplyDeadband(std::clamp(msg->linear.y, -max_vy_, max_vy_), deadband_);
  const double wz = ApplyDeadband(std::clamp(msg->angular.z, -max_wz_, max_wz_), deadband_);
  const bool zero_command = IsZeroVelocity(vx, vy, wz);

  if (zero_command) {
    last_cmd_time_ = SteadyClock::now();
    ClearCommandLocked();
    PublishStopLocked("zero cmd_vel");
    return;
  }

  if (!armed_) {
    if (moving_ || received_cmd_) {
      ClearCommandLocked();
      PublishStopLocked("cmd_vel received while disarmed");
    }
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kLogThrottleDuration,
                         "Ignored non-zero cmd_vel because the bridge is DISARMED.");
    return;
  }

  target_vx_ = vx;
  target_vy_ = vy;
  target_wz_ = wz;
  last_cmd_time_ = SteadyClock::now();
  received_cmd_ = true;
  moving_ = true;

  if (!PublishVelocityLocked(target_vx_, target_vy_, target_wz_)) {
    ClearCommandLocked();
    return;
  }

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), kLogThrottleDuration,
                       "Forwarding velocity: vx=%.3f m/s, vy=%.3f m/s, wz=%.3f rad/s", target_vx_,
                       target_vy_, target_wz_);
}

void G1TwistBridge::ResponseCallback(const unitree_api::msg::Response::ConstSharedPtr msg) {
  if (msg->header.identity.api_id != kSetVelocityApiId) {
    return;
  }

  if (msg->header.status.code != 0) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), kLogThrottleDuration,
                          "SetVelocity response reported failure (request=%lld, status=%d).",
                          static_cast<long long>(msg->header.identity.id),
                          static_cast<int>(msg->header.status.code));
    return;
  }

  RCLCPP_DEBUG(get_logger(), "SetVelocity response received for request %lld.",
               static_cast<long long>(msg->header.identity.id));
}

void G1TwistBridge::SendTimerCallback() {
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (!armed_ || !received_cmd_) {
    return;
  }

  const double elapsed = std::chrono::duration<double>(SteadyClock::now() - last_cmd_time_).count();
  if (elapsed > command_timeout_) {
    ClearCommandLocked();
    PublishStopLocked("cmd_vel timeout");
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kLogThrottleDuration,
                         "cmd_vel timed out after %.3f s; sent zero velocity.", elapsed);
    return;
  }

  if (!PublishVelocityLocked(target_vx_, target_vy_, target_wz_)) {
    ClearCommandLocked();
  }
}

rcl_interfaces::msg::SetParametersResult G1TwistBridge::OnSetParameters(
    const std::vector<rclcpp::Parameter>& parameters) {
  std::lock_guard<std::mutex> lock(state_mutex_);

  bool next_armed = armed_;
  double next_max_vx = max_vx_;
  double next_max_vy = max_vy_;
  double next_max_wz = max_wz_;
  double next_deadband = deadband_;
  double next_command_duration = command_duration_;
  double next_command_timeout = command_timeout_;
  bool runtime_configuration_changed = false;

  try {
    for (const auto& parameter : parameters) {
      const std::string& name = parameter.get_name();
      if (name == "cmd_vel_topic" || name == "request_topic" || name == "response_topic" ||
          name == "send_frequency") {
        return FailedParameterResult(name + " cannot be changed while the node is running");
      }
      if (name == "armed") {
        next_armed = parameter.as_bool();
      } else if (name == "max_vx") {
        next_max_vx = parameter.as_double();
        runtime_configuration_changed = true;
      } else if (name == "max_vy") {
        next_max_vy = parameter.as_double();
        runtime_configuration_changed = true;
      } else if (name == "max_wz") {
        next_max_wz = parameter.as_double();
        runtime_configuration_changed = true;
      } else if (name == "deadband") {
        next_deadband = parameter.as_double();
        runtime_configuration_changed = true;
      } else if (name == "command_duration") {
        next_command_duration = parameter.as_double();
        runtime_configuration_changed = true;
      } else if (name == "command_timeout") {
        next_command_timeout = parameter.as_double();
        runtime_configuration_changed = true;
      }
    }
  } catch (const rclcpp::ParameterTypeException& exception) {
    return FailedParameterResult(exception.what());
  }

  const std::string validation_error = ValidateConfiguration(
      cmd_vel_topic_, request_topic_, response_topic_, next_max_vx, next_max_vy, next_max_wz,
      next_deadband, send_frequency_, next_command_duration, next_command_timeout);
  if (!validation_error.empty()) {
    return FailedParameterResult(validation_error);
  }

  const bool was_armed = armed_;
  const bool disarming = was_armed && !next_armed;
  const bool arming = !was_armed && next_armed;
  armed_ = next_armed;
  max_vx_ = next_max_vx;
  max_vy_ = next_max_vy;
  max_wz_ = next_max_wz;
  deadband_ = next_deadband;
  command_duration_ = next_command_duration;
  command_timeout_ = next_command_timeout;

  if (disarming) {
    ClearCommandLocked();
    PublishStopLocked("armed set to false");
    RCLCPP_WARN(get_logger(), "Bridge DISARMED; sent zero velocity immediately.");
  } else if (runtime_configuration_changed && received_cmd_) {
    target_vx_ = ApplyDeadband(std::clamp(target_vx_, -max_vx_, max_vx_), deadband_);
    target_vy_ = ApplyDeadband(std::clamp(target_vy_, -max_vy_, max_vy_), deadband_);
    target_wz_ = ApplyDeadband(std::clamp(target_wz_, -max_wz_, max_wz_), deadband_);

    const double elapsed =
        std::chrono::duration<double>(SteadyClock::now() - last_cmd_time_).count();
    if (!armed_ || IsZeroVelocity(target_vx_, target_vy_, target_wz_) ||
        elapsed > command_timeout_) {
      ClearCommandLocked();
      PublishStopLocked("runtime parameter update");
    } else if (!PublishVelocityLocked(target_vx_, target_vy_, target_wz_)) {
      ClearCommandLocked();
    }
  }

  if (arming) {
    RCLCPP_INFO(get_logger(), "Bridge ARMED; valid cmd_vel commands may now be sent.");
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

bool G1TwistBridge::PublishVelocityLocked(double vx, double vy, double wz) noexcept {
  try {
    unitree_api::msg::Request request;
    request.header.identity.id = next_request_id_.fetch_add(1, std::memory_order_relaxed);
    request.header.identity.api_id = kSetVelocityApiId;
    request.parameter = VelocityJson(vx, vy, wz, command_duration_);
    request_pub_->publish(request);
    return true;
  } catch (const std::exception& exception) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), kLogThrottleDuration,
                          "Failed to publish Unitree SetVelocity request: %s", exception.what());
  } catch (...) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), kLogThrottleDuration,
                          "Failed to publish Unitree SetVelocity request: unknown exception");
  }
  return false;
}

void G1TwistBridge::PublishStopLocked(const char* reason) noexcept {
  if (PublishVelocityLocked(0.0, 0.0, 0.0)) {
    RCLCPP_DEBUG(get_logger(), "Published zero velocity: %s", reason);
  }
}

void G1TwistBridge::ClearCommandLocked() noexcept {
  target_vx_ = 0.0;
  target_vy_ = 0.0;
  target_wz_ = 0.0;
  moving_ = false;
  received_cmd_ = false;
}

}  // namespace g1_twist_bridge
