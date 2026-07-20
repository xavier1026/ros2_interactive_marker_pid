#include "pid_goal_controller/pid_goal_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <mutex>

#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pid_goal_controller {
namespace {

constexpr char kDefaultGoalTopic[] = "/goal_pose";
constexpr char kDefaultGlobalFrame[] = "map";
constexpr char kDefaultBaseFrame[] = "base_link";
constexpr char kDefaultCmdVelTopic[] = "/cmd_vel";
constexpr char kDefaultErrorTopic[] = "/pid_error";
constexpr char kDefaultDistanceTopic[] = "/pid_distance";

constexpr double kDefaultMaxV = 0.10;
constexpr double kDefaultMaxW = 0.15;
constexpr double kDefaultPosTolerance = 0.10;
constexpr double kDefaultYawTolerance = 0.10;

constexpr double kMaxControlDt = 0.2;
constexpr double kMaxIntegral = 0.5;

double ClampValue(double value, double low, double high) {
  return std::max(low, std::min(value, high));
}

double WrapAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }

  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }

  return angle;
}

double YawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);

  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);

  return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace

PidGoalController::PidGoalController() : Node("pid_goal_controller") {
  declare_parameter<double>("kp_x", 1.0);
  declare_parameter<double>("ki_x", 0.0);
  declare_parameter<double>("kd_x", 0.05);
  declare_parameter<double>("kp_y", 1.0);
  declare_parameter<double>("ki_y", 0.0);
  declare_parameter<double>("kd_y", 0.05);
  declare_parameter<double>("kp_yaw", 2.0);
  declare_parameter<double>("ki_yaw", 0.0);
  declare_parameter<double>("kd_yaw", 0.08);
  declare_parameter<double>("max_v", kDefaultMaxV);
  declare_parameter<double>("max_w", kDefaultMaxW);
  declare_parameter<double>("pos_tolerance", kDefaultPosTolerance);
  declare_parameter<double>("yaw_tolerance", kDefaultYawTolerance);
  declare_parameter<double>("control_frequency", 30.0);
  declare_parameter<double>("transform_timeout", 0.1);
  declare_parameter<bool>("publish_debug", true);

  declare_parameter<std::string>("goal_topic", kDefaultGoalTopic);
  declare_parameter<std::string>("global_frame", kDefaultGlobalFrame);
  declare_parameter<std::string>("base_frame", kDefaultBaseFrame);
  declare_parameter<std::string>("cmd_vel_topic", kDefaultCmdVelTopic);
  declare_parameter<std::string>("error_topic", kDefaultErrorTopic);
  declare_parameter<std::string>("distance_topic", kDefaultDistanceTopic);

  max_v_ = std::fabs(get_parameter("max_v").as_double());
  max_w_ = std::fabs(get_parameter("max_w").as_double());
  pos_tolerance_ = std::fabs(get_parameter("pos_tolerance").as_double());
  yaw_tolerance_ = std::fabs(get_parameter("yaw_tolerance").as_double());
  control_frequency_ = get_parameter("control_frequency").as_double();
  transform_timeout_ = get_parameter("transform_timeout").as_double();
  publish_debug_ = get_parameter("publish_debug").as_bool();

  if (control_frequency_ <= 0.0) {
    RCLCPP_WARN(get_logger(),
                "control_frequency must be positive. "
                "Falling back to 30 Hz.");
    control_frequency_ = 30.0;
  }

  if (transform_timeout_ <= 0.0) {
    RCLCPP_WARN(get_logger(),
                "transform_timeout must be positive. "
                "Falling back to 0.1 s.");
    transform_timeout_ = 0.1;
  }

  goal_topic_ = get_parameter("goal_topic").as_string();
  global_frame_ = get_parameter("global_frame").as_string();
  base_frame_ = get_parameter("base_frame").as_string();
  cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
  error_topic_ = get_parameter("error_topic").as_string();
  distance_topic_ = get_parameter("distance_topic").as_string();

  pid_x_ = Pid(get_parameter("kp_x").as_double(), get_parameter("ki_x").as_double(),
               get_parameter("kd_x").as_double(), max_v_, kMaxIntegral);
  pid_y_ = Pid(get_parameter("kp_y").as_double(), get_parameter("ki_y").as_double(),
               get_parameter("kd_y").as_double(), max_v_, kMaxIntegral);
  pid_yaw_ = Pid(get_parameter("kp_yaw").as_double(), get_parameter("ki_yaw").as_double(),
                 get_parameter("kd_yaw").as_double(), max_w_, kMaxIntegral);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  error_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(error_topic_, 10);
  distance_pub_ = create_publisher<std_msgs::msg::Float64>(distance_topic_, 10);
  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, 1, std::bind(&PidGoalController::GoalCallback, this, std::placeholders::_1));

  last_time_ = std::chrono::steady_clock::now();

  const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / control_frequency_));

  timer_ = create_wall_timer(timer_period, std::bind(&PidGoalController::ControlLoop, this));

  RCLCPP_INFO(get_logger(), "pid_goal_controller started.");
  RCLCPP_INFO(get_logger(), "goal_topic=%s global_frame=%s base_frame=%s cmd_vel_topic=%s",
              goal_topic_.c_str(), global_frame_.c_str(), base_frame_.c_str(),
              cmd_vel_topic_.c_str());
  RCLCPP_INFO(get_logger(), "error_topic=%s distance_topic=%s", error_topic_.c_str(),
              distance_topic_.c_str());
}

PidGoalController::~PidGoalController() noexcept {
  try {
    if (cmd_pub_) {
      PublishStop();
    }
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(get_logger(), "Failed to publish zero Twist while shutting down: %s", ex.what());
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "Failed to publish zero Twist while shutting down.");
  }
}

void PidGoalController::GoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  if (msg->header.frame_id.empty()) {
    RejectGoal();
    RCLCPP_WARN(get_logger(),
                "Received goal with empty frame_id. Rejecting it and publishing zero Twist.");
    return;
  }

  geometry_msgs::msg::PoseStamped goal_in_global;

  if (msg->header.frame_id == global_frame_) {
    goal_in_global = *msg;
  } else {
    try {
      // Buffer::transform uses the PoseStamped timestamp (or the latest TF for a zero stamp).
      tf_buffer_->transform(*msg, goal_in_global, global_frame_,
                            tf2::durationFromSec(transform_timeout_));
    } catch (const tf2::TransformException& ex) {
      RejectGoal();
      RCLCPP_WARN(get_logger(),
                  "Failed to transform goal from '%s' to '%s': %s. "
                  "Rejecting it and publishing zero Twist.",
                  msg->header.frame_id.c_str(), global_frame_.c_str(), ex.what());
      return;
    }
  }

  {
    std::lock_guard<std::mutex> lock(mtx_);

    goal_pose_ = goal_in_global;
    have_goal_ = true;
    ++goal_generation_;
    ResetPid();
    last_time_ = std::chrono::steady_clock::now();
  }

  RCLCPP_INFO(get_logger(), "Accepted goal in '%s' (received in '%s').", global_frame_.c_str(),
              msg->header.frame_id.c_str());
}

void PidGoalController::PublishDebug(double error_x_body, double error_y_body, double error_yaw,
                                     double distance) {
  geometry_msgs::msg::TwistStamped error_msg;

  error_msg.header.frame_id = base_frame_;
  error_msg.header.stamp = now();
  error_msg.twist.linear.x = error_x_body;
  error_msg.twist.linear.y = error_y_body;
  error_msg.twist.angular.z = error_yaw;
  error_pub_->publish(error_msg);

  std_msgs::msg::Float64 distance_msg;
  distance_msg.data = distance;
  distance_pub_->publish(distance_msg);
}

void PidGoalController::PublishStop() {
  geometry_msgs::msg::Twist cmd;

  cmd.linear.x = 0.0;
  cmd.linear.y = 0.0;
  cmd.linear.z = 0.0;
  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;
  cmd.angular.z = 0.0;
  cmd_pub_->publish(cmd);
}

void PidGoalController::RejectGoal() {
  {
    std::lock_guard<std::mutex> lock(mtx_);
    have_goal_ = false;
    ++goal_generation_;
    ResetPid();
  }

  PublishStop();
}

void PidGoalController::ResetPid() {
  pid_x_.Reset();
  pid_y_.Reset();
  pid_yaw_.Reset();
}

void PidGoalController::ControlLoop() {
  geometry_msgs::msg::PoseStamped goal_pose;
  std::uint64_t goal_generation = 0;
  double dt = 0.0;

  /*
   * 锁内只复制目标和更新时间。
   * TF查询可能等待，不能拿着锁查询TF，
   * 否则会阻塞GoalCallback更新最新目标。
   */
  {
    std::lock_guard<std::mutex> lock(mtx_);

    if (!have_goal_) {
      return;
    }

    goal_pose = goal_pose_;
    goal_generation = goal_generation_;
    const auto current_time = std::chrono::steady_clock::now();
    dt = std::chrono::duration<double>(current_time - last_time_).count();
    last_time_ = current_time;
  }

  if (dt <= 0.0) {
    return;
  }

  bool reset_pid = false;

  if (dt > kMaxControlDt) {
    dt = kMaxControlDt;
    reset_pid = true;
  }

  geometry_msgs::msg::TransformStamped robot_in_global;

  try {
    // lookupTransform(target, source, ...) returns the source frame pose in the target frame.
    robot_in_global = tf_buffer_->lookupTransform(global_frame_, base_frame_, tf2::TimePointZero,
                                                  tf2::durationFromSec(transform_timeout_));
  } catch (const tf2::TransformException& ex) {
    {
      std::lock_guard<std::mutex> lock(mtx_);
      ResetPid();
    }

    PublishStop();

    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Required TF '%s' -> '%s' is unavailable: %s. "
                         "Publishing zero Twist.",
                         global_frame_.c_str(), base_frame_.c_str(), ex.what());

    return;
  }

  /*
   * goal_pose_ is always expressed in global_frame_. The transform gives the
   * current base_frame_ pose in that same global frame. Compute the world-frame
   * error first, then rotate it into the robot body frame for velocity control.
   */
  const double robot_x = robot_in_global.transform.translation.x;
  const double robot_y = robot_in_global.transform.translation.y;
  const double robot_yaw = YawFromQuaternion(robot_in_global.transform.rotation);
  const double goal_yaw = YawFromQuaternion(goal_pose.pose.orientation);

  const double error_x_world = goal_pose.pose.position.x - robot_x;
  const double error_y_world = goal_pose.pose.position.y - robot_y;
  const double error_yaw = WrapAngle(goal_yaw - robot_yaw);

  const double cos_yaw = std::cos(robot_yaw);
  const double sin_yaw = std::sin(robot_yaw);
  const double error_x_body = cos_yaw * error_x_world + sin_yaw * error_y_world;
  const double error_y_body = -sin_yaw * error_x_world + cos_yaw * error_y_world;
  const double distance = std::hypot(error_x_world, error_y_world);

  if (publish_debug_) {
    PublishDebug(error_x_body, error_y_body, error_yaw, distance);
  }

  if (distance <= pos_tolerance_ && std::fabs(error_yaw) <= yaw_tolerance_) {
    {
      std::lock_guard<std::mutex> lock(mtx_);

      if (!have_goal_ || goal_generation != goal_generation_) {
        return;
      }

      have_goal_ = false;
      ++goal_generation_;
      ResetPid();
      PublishStop();
    }

    RCLCPP_INFO(get_logger(), "Goal reached. Published zero Twist.");
    return;
  }

  geometry_msgs::msg::Twist cmd;

  {
    std::lock_guard<std::mutex> lock(mtx_);

    if (!have_goal_ || goal_generation != goal_generation_) {
      return;
    }

    if (reset_pid) {
      ResetPid();
    }

    if (distance > pos_tolerance_) {
      double vx = pid_x_.Update(error_x_body, dt);
      double vy = pid_y_.Update(error_y_body, dt);
      const double v_norm = std::hypot(vx, vy);

      if (v_norm > max_v_ && v_norm > 0.0) {
        const double scale = max_v_ / v_norm;
        vx *= scale;
        vy *= scale;
      }

      cmd.linear.x = vx;
      cmd.linear.y = vy;
    } else {
      pid_x_.Reset();
      pid_y_.Reset();
      cmd.linear.x = 0.0;
      cmd.linear.y = 0.0;
    }

    if (std::fabs(error_yaw) > yaw_tolerance_) {
      cmd.angular.z = ClampValue(pid_yaw_.Update(error_yaw, dt), -max_w_, max_w_);
    } else {
      pid_yaw_.Reset();
      cmd.angular.z = 0.0;
    }

    // Keep publication ordered with goal invalidation in a multi-threaded executor.
    cmd_pub_->publish(cmd);
  }
}

}  // namespace pid_goal_controller
