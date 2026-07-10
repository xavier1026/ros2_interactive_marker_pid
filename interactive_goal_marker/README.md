# interactive_goal_marker

`interactive_goal_marker` is a reusable ROS 2 C++ package that publishes a goal pose from an RViz2 Interactive Marker.

## Interface

Input:

- RViz2 Interactive Marker interaction.

Output:

- `geometry_msgs/msg/PoseStamped`
- Default topic: `goal_pose`

This node does not compute control commands and does not publish `cmd_vel`.

## Parameters

- `frame_id` (`string`, default: `odom`): frame used for the marker and published goal.
- `server_name` (`string`, default: `goal_marker`): Interactive Marker Server name.
- `goal_topic` (`string`, default: `goal_pose`): topic used to publish goal poses.
- `use_sim_time` (`bool`, default: `false`): use simulation time.

## Run

```bash
ros2 launch interactive_goal_marker interactive_goal_marker.launch.py
```
