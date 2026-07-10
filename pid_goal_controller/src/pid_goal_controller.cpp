#include "pid_goal_controller/pid_goal_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>

namespace pid_goal_controller {
namespace {

constexpr char kDefaultGoalTopic[] = "goal_pose";
constexpr char kDefaultOdomTopic[] = "odom";
constexpr char kDefaultCmdVelTopic[] = "cmd_vel";
constexpr char kDefaultErrorTopic[] = "pid_error";
constexpr char kDefaultDistanceTopic[] = "pid_distance";
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

  declare_parameter<double>("max_v", 0.4);
  declare_parameter<double>("max_w", 1.0);

  declare_parameter<double>("pos_tolerance", 0.03);
  declare_parameter<double>("yaw_tolerance", 0.05);
  declare_parameter<double>("control_frequency", 30.0);
  declare_parameter<double>("odom_timeout", 0.5);

  declare_parameter<std::string>("goal_topic", kDefaultGoalTopic);
  declare_parameter<std::string>("odom_topic", kDefaultOdomTopic);
  declare_parameter<std::string>("cmd_vel_topic", kDefaultCmdVelTopic);
  declare_parameter<std::string>("error_topic", kDefaultErrorTopic);
  declare_parameter<std::string>("distance_topic", kDefaultDistanceTopic);

  max_v_ = std::fabs(get_parameter("max_v").as_double());
  max_w_ = std::fabs(get_parameter("max_w").as_double());
  pos_tolerance_ = std::fabs(get_parameter("pos_tolerance").as_double());
  yaw_tolerance_ = std::fabs(get_parameter("yaw_tolerance").as_double());
  control_frequency_ = get_parameter("control_frequency").as_double();
  odom_timeout_ = get_parameter("odom_timeout").as_double();

  if (control_frequency_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "control_frequency must be positive. Falling back to 30 Hz.");
    control_frequency_ = 30.0;
  }
  if (odom_timeout_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "odom_timeout must be positive. Falling back to 0.5 s.");
    odom_timeout_ = 0.5;
  }

  goal_topic_ = get_parameter("goal_topic").as_string();
  odom_topic_ = get_parameter("odom_topic").as_string();
  cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
  error_topic_ = get_parameter("error_topic").as_string();
  distance_topic_ = get_parameter("distance_topic").as_string();

  pid_x_ = Pid(get_parameter("kp_x").as_double(), get_parameter("ki_x").as_double(),
               get_parameter("kd_x").as_double(), max_v_, kMaxIntegral);
  pid_y_ = Pid(get_parameter("kp_y").as_double(), get_parameter("ki_y").as_double(),
               get_parameter("kd_y").as_double(), max_v_, kMaxIntegral);
  pid_yaw_ = Pid(get_parameter("kp_yaw").as_double(), get_parameter("ki_yaw").as_double(),
                 get_parameter("kd_yaw").as_double(), max_w_, kMaxIntegral);

  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  error_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(error_topic_, 10);
  distance_pub_ = create_publisher<std_msgs::msg::Float64>(distance_topic_, 10);

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10, std::bind(&PidGoalController::OdomCallback, this, std::placeholders::_1));
  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, 10, std::bind(&PidGoalController::GoalCallback, this, std::placeholders::_1));

  last_time_ = now();
  last_odom_wall_time_ = std::chrono::steady_clock::now();

  const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / control_frequency_));
  timer_ = create_wall_timer(timer_period, std::bind(&PidGoalController::ControlLoop, this));

  RCLCPP_INFO(get_logger(), "pid_goal_controller started.");
  RCLCPP_INFO(get_logger(), "goal_topic=%s odom_topic=%s cmd_vel_topic=%s", goal_topic_.c_str(),
              odom_topic_.c_str(), cmd_vel_topic_.c_str());
  RCLCPP_INFO(get_logger(), "error_topic=%s distance_topic=%s", error_topic_.c_str(),
              distance_topic_.c_str());
}

void PidGoalController::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  x_ = msg->pose.pose.position.x;
  y_ = msg->pose.pose.position.y;
  yaw_ = YawFromQuaternion(msg->pose.pose.orientation);

  last_odom_wall_time_ = std::chrono::steady_clock::now();
  have_odom_ = true;
}

void PidGoalController::GoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  goal_x_ = msg->pose.position.x;
  goal_y_ = msg->pose.position.y;
  goal_yaw_ = YawFromQuaternion(msg->pose.orientation);

  have_goal_ = true;
  last_time_ = now();
  ResetPid();

  RCLCPP_INFO(get_logger(), "New goal: x=%.3f, y=%.3f, yaw=%.3f", goal_x_, goal_y_, goal_yaw_);
}

void PidGoalController::PublishDebug(double error_x_body, double error_y_body, double error_yaw,
                                     double distance) {
  geometry_msgs::msg::TwistStamped error_msg;
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
  cmd_pub_->publish(cmd);
}

void PidGoalController::ResetPid() {
  pid_x_.Reset();
  pid_y_.Reset();
  pid_yaw_.Reset();
}

void PidGoalController::ControlLoop() {
  if (!have_goal_) {
    return;
  }

  if (!have_odom_) {
    PublishStop();
    return;
  }

  const auto now_wall_time = std::chrono::steady_clock::now();
  const double odom_age =
      std::chrono::duration<double>(now_wall_time - last_odom_wall_time_).count();
  if (odom_age > odom_timeout_) {
    PublishStop();
    ResetPid();
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "No odom received for %.3f s. Publishing zero cmd_vel.", odom_age);
    return;
  }

  const rclcpp::Time now_time = now();
  double dt = (now_time - last_time_).seconds();
  last_time_ = now_time;

  if (dt <= 0.0) {
    return;
  }

  if (dt > kMaxControlDt) {
    ResetPid();
    dt = kMaxControlDt;
  }

  const double error_x_world = goal_x_ - x_;
  const double error_y_world = goal_y_ - y_;
  const double error_yaw = WrapAngle(goal_yaw_ - yaw_);
  const double distance = std::hypot(error_x_world, error_y_world);

  const double cos_yaw = std::cos(yaw_);
  const double sin_yaw = std::sin(yaw_);
  const double error_x_body = cos_yaw * error_x_world + sin_yaw * error_y_world;
  const double error_y_body = -sin_yaw * error_x_world + cos_yaw * error_y_world;

  PublishDebug(error_x_body, error_y_body, error_yaw, distance);

  if (distance <= pos_tolerance_ && std::fabs(error_yaw) <= yaw_tolerance_) {
    PublishStop();
    have_goal_ = false;
    ResetPid();
    RCLCPP_INFO(get_logger(), "Goal reached.");
    return;
  }

  geometry_msgs::msg::Twist cmd;

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
  }

  if (std::fabs(error_yaw) > yaw_tolerance_) {
    cmd.angular.z = ClampValue(pid_yaw_.Update(error_yaw, dt), -max_w_, max_w_);
  } else {
    pid_yaw_.Reset();
  }

  cmd_pub_->publish(cmd);
}

}  // namespace pid_goal_controller
