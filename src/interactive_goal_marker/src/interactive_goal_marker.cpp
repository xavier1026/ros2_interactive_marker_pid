#include "interactive_goal_marker/interactive_goal_marker.hpp"

#include <functional>

#include "visualization_msgs/msg/interactive_marker_control.hpp"

namespace interactive_goal_marker {
namespace {

constexpr char kDefaultFrameId[] = "map";
constexpr char kDefaultServerName[] = "goal_marker";
constexpr char kDefaultGoalTopic[] = "/goal_pose";

constexpr char kMarkerName[] = "target_goal";
constexpr double kInteractiveMarkerScale = 0.45;
constexpr double kUnitQuaternionW = 1.0;

constexpr double kTargetDiskDiameter = 0.25;
constexpr double kTargetDiskHeight = 0.025;
constexpr double kTargetDiskZ = 0.015;

constexpr double kCenterSphereDiameter = 0.12;
constexpr double kCenterSphereZ = 0.08;

constexpr double kDirectionArrowLength = 0.35;
constexpr double kDirectionArrowShaftDiameter = 0.05;
constexpr double kDirectionArrowX = 0.13;
constexpr double kDirectionArrowZ = 0.10;

}  // namespace

InteractiveGoalMarker::InteractiveGoalMarker() : Node("interactive_goal_marker") {
  declare_parameter<std::string>("frame_id", kDefaultFrameId);
  declare_parameter<std::string>("server_name", kDefaultServerName);
  declare_parameter<std::string>("goal_topic", kDefaultGoalTopic);

  frame_id_ = get_parameter("frame_id").as_string();
  if (frame_id_ != kDefaultFrameId) {
    RCLCPP_WARN(get_logger(),
                "frame_id='%s' is not supported; using the required global frame '%s'.",
                frame_id_.c_str(), kDefaultFrameId);
    frame_id_ = kDefaultFrameId;
  }
  server_name_ = get_parameter("server_name").as_string();
  goal_topic_ = get_parameter("goal_topic").as_string();

  goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic_, 10);

  server_ = std::make_unique<interactive_markers::InteractiveMarkerServer>(
      server_name_, get_node_base_interface(), get_node_clock_interface(),
      get_node_logging_interface(), get_node_topics_interface(), get_node_services_interface());

  MakeGoalMarker();

  RCLCPP_INFO(get_logger(), "interactive_goal_marker started.");
  RCLCPP_INFO(get_logger(), "RViz namespace: %s", server_name_.c_str());
  RCLCPP_INFO(get_logger(), "The goal is published once on %s when the mouse button is released.",
              goal_topic_.c_str());
  RCLCPP_INFO(get_logger(), "This node only publishes %s. It does not publish /cmd_vel.",
              goal_topic_.c_str());
}

visualization_msgs::msg::Marker InteractiveGoalMarker::MakeTargetDisk() const {
  visualization_msgs::msg::Marker marker;

  marker.type = visualization_msgs::msg::Marker::CYLINDER;
  marker.scale.x = kTargetDiskDiameter;
  marker.scale.y = kTargetDiskDiameter;
  marker.scale.z = kTargetDiskHeight;

  marker.color.r = 1.0;
  marker.color.g = 0.75;
  marker.color.b = 0.05;
  marker.color.a = 0.85;

  marker.pose.position.z = kTargetDiskZ;
  marker.pose.orientation.w = kUnitQuaternionW;

  return marker;
}

visualization_msgs::msg::Marker InteractiveGoalMarker::MakeCenterSphere() const {
  visualization_msgs::msg::Marker marker;

  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.scale.x = kCenterSphereDiameter;
  marker.scale.y = kCenterSphereDiameter;
  marker.scale.z = kCenterSphereDiameter;

  marker.color.r = 0.05;
  marker.color.g = 0.25;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  marker.pose.position.z = kCenterSphereZ;
  marker.pose.orientation.w = kUnitQuaternionW;

  return marker;
}

visualization_msgs::msg::Marker InteractiveGoalMarker::MakeDirectionArrow() const {
  visualization_msgs::msg::Marker marker;

  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.scale.x = kDirectionArrowLength;
  marker.scale.y = kDirectionArrowShaftDiameter;
  marker.scale.z = kDirectionArrowShaftDiameter;

  marker.color.r = 0.05;
  marker.color.g = 0.85;
  marker.color.b = 0.20;
  marker.color.a = 1.0;

  marker.pose.position.x = kDirectionArrowX;
  marker.pose.position.z = kDirectionArrowZ;
  marker.pose.orientation.w = kUnitQuaternionW;

  return marker;
}

void InteractiveGoalMarker::AddDragMarkerControl(
    visualization_msgs::msg::InteractiveMarker* marker) const {
  visualization_msgs::msg::InteractiveMarkerControl control;

  control.name = "drag_target_marker";
  control.always_visible = true;
  control.orientation.w = kUnitQuaternionW;
  control.orientation.y = 1.0;
  control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;

  control.markers.push_back(MakeTargetDisk());
  control.markers.push_back(MakeCenterSphere());
  control.markers.push_back(MakeDirectionArrow());

  marker->controls.push_back(control);
}

void InteractiveGoalMarker::AddMoveXControl(
    visualization_msgs::msg::InteractiveMarker* marker) const {
  visualization_msgs::msg::InteractiveMarkerControl control;

  control.name = "move_x_axis";
  control.orientation.w = kUnitQuaternionW;
  control.orientation.x = 1.0;
  control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;

  marker->controls.push_back(control);
}

void InteractiveGoalMarker::AddMoveYControl(
    visualization_msgs::msg::InteractiveMarker* marker) const {
  visualization_msgs::msg::InteractiveMarkerControl control;

  control.name = "move_y_axis";
  control.orientation.w = kUnitQuaternionW;
  control.orientation.z = 1.0;
  control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;

  marker->controls.push_back(control);
}

void InteractiveGoalMarker::AddRotateZControl(
    visualization_msgs::msg::InteractiveMarker* marker) const {
  visualization_msgs::msg::InteractiveMarkerControl control;

  control.name = "rotate_z_axis";
  control.orientation.w = kUnitQuaternionW;
  control.orientation.y = 1.0;
  control.orientation_mode = visualization_msgs::msg::InteractiveMarkerControl::FIXED;
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;

  marker->controls.push_back(control);
}

void InteractiveGoalMarker::MakeGoalMarker() {
  visualization_msgs::msg::InteractiveMarker marker;

  marker.header.frame_id = frame_id_;
  marker.header.stamp = now();
  marker.name = kMarkerName;
  marker.description = "Drag target marker to choose goal";
  marker.scale = kInteractiveMarkerScale;

  marker.pose.position.z = 0.0;
  marker.pose.orientation.w = kUnitQuaternionW;

  AddDragMarkerControl(&marker);
  AddMoveXControl(&marker);
  AddMoveYControl(&marker);
  AddRotateZControl(&marker);

  server_->insert(marker,
                  std::bind(&InteractiveGoalMarker::ProcessFeedback, this, std::placeholders::_1));
  server_->applyChanges();
}

void InteractiveGoalMarker::PublishGoal(const geometry_msgs::msg::Pose& pose) {
  geometry_msgs::msg::PoseStamped goal_msg;

  goal_msg.header.frame_id = frame_id_;
  goal_msg.header.stamp = now();
  goal_msg.pose = pose;
  goal_msg.pose.position.z = 0.0;

  goal_pub_->publish(goal_msg);

  RCLCPP_INFO(get_logger(), "Published %s: x=%.3f, y=%.3f, z=%.3f, qz=%.3f, qw=%.3f",
              goal_topic_.c_str(), goal_msg.pose.position.x, goal_msg.pose.position.y,
              goal_msg.pose.position.z, goal_msg.pose.orientation.z, goal_msg.pose.orientation.w);
}

void InteractiveGoalMarker::ProcessFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback) {
  if (feedback->event_type != visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE &&
      feedback->event_type != visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP) {
    return;
  }

  geometry_msgs::msg::Pose pose = feedback->pose;
  pose.position.z = 0.0;

  server_->setPose(feedback->marker_name, pose);
  server_->applyChanges();

  if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP) {
    PublishGoal(pose);
  }
}

}  // namespace interactive_goal_marker
