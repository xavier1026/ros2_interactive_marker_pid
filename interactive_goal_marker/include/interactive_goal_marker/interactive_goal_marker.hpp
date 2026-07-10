#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "interactive_markers/interactive_marker_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace interactive_goal_marker {

class InteractiveGoalMarker : public rclcpp::Node {
 public:
  InteractiveGoalMarker();

 private:
  visualization_msgs::msg::Marker MakeTargetDisk() const;
  visualization_msgs::msg::Marker MakeCenterSphere() const;
  visualization_msgs::msg::Marker MakeDirectionArrow() const;

  void AddDragMarkerControl(visualization_msgs::msg::InteractiveMarker* marker) const;
  void AddMoveXControl(visualization_msgs::msg::InteractiveMarker* marker) const;
  void AddMoveYControl(visualization_msgs::msg::InteractiveMarker* marker) const;
  void AddRotateZControl(visualization_msgs::msg::InteractiveMarker* marker) const;

  void MakeGoalMarker();
  void PublishGoal(const geometry_msgs::msg::Pose& pose);
  void ProcessFeedback(
      const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback);

  std::string frame_id_;
  std::string server_name_;
  std::string goal_topic_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
};

}  // namespace interactive_goal_marker
