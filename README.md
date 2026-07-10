# ROS2 Interactive Marker and PID Goal Controller

本仓库包含两个独立的 ROS2 Humble C++ package，用于实现 RViz2 目标输入和底盘 PID 点到点控制。

## Packages

### interactive_goal_marker

在 RViz2 中提供可平移、可旋转的 Interactive Marker，并发布目标位姿。

- 输出话题：`goal_pose`
- 消息类型：`geometry_msgs/msg/PoseStamped`

该节点只负责生成目标位姿，不负责计算底盘速度。

### pid_goal_controller

订阅目标位姿和里程计，计算机器人坐标系下的位置误差与航向误差，并通过 PID 发布底盘速度指令。

输入：

- `goal_pose`：`geometry_msgs/msg/PoseStamped`
- `odom`：`nav_msgs/msg/Odometry`

输出：

- `cmd_vel`：`geometry_msgs/msg/Twist`
- `pid_error`：`geometry_msgs/msg/TwistStamped`
- `pid_distance`：`std_msgs/msg/Float64`

## Data Flow

```text
RViz2 Interactive Marker
          |
          | goal_pose
          v
PID Goal Controller <--- odom
          |
          | cmd_vel
          v
Robot Base / Simulator
```

## Build

将本仓库中的两个 package 放入 ROS2 工作空间的 `src` 目录后执行：

```bash
source /opt/ros/humble/setup.bash
cd ~/Robot/ros2_ws

colcon build \
  --packages-select interactive_goal_marker pid_goal_controller \
  --symlink-install

source install/setup.bash
```

## Run

启动 Interactive Marker：

```bash
ros2 launch interactive_goal_marker interactive_goal_marker.launch.py
```

启动 PID 控制器：

```bash
ros2 launch pid_goal_controller pid_goal_controller.launch.py
```

具体接口、参数及测试方法见两个 package 内各自的 `README.md`。
