# pid_goal_controller

`pid_goal_controller` is a reusable ROS 2 C++ package that converts a goal pose and odometry into planar `cmd_vel` commands for an omnidirectional base.

## Interface

Subscriptions:

- `geometry_msgs/msg/PoseStamped` on `goal_pose` by default.
- `nav_msgs/msg/Odometry` on `odom` by default.

Publications:

- `geometry_msgs/msg/Twist` on `cmd_vel` by default.
- `geometry_msgs/msg/TwistStamped` on `pid_error` by default.
- `std_msgs/msg/Float64` on `pid_distance` by default.

`pid_error` is a debug topic for PlotJuggler, not a velocity command:

- `twist.linear.x = error_x_body`
- `twist.linear.y = error_y_body`
- `twist.angular.z = error_yaw`

`pid_distance.data = sqrt(error_x_world^2 + error_y_world^2)`.

## Parameters

Topic parameters:

- `goal_topic` (`string`, default: `goal_pose`)
- `odom_topic` (`string`, default: `odom`)
- `cmd_vel_topic` (`string`, default: `cmd_vel`)
- `error_topic` (`string`, default: `pid_error`)
- `distance_topic` (`string`, default: `pid_distance`)

Control parameters are loaded from `config/pid_goal_controller.yaml` by the launch file.

## Run

```bash
ros2 launch pid_goal_controller pid_goal_controller.launch.py
```
