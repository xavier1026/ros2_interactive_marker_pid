# ROS2 Interactive Marker PID Workspace

本仓库是一个 ROS2 Humble colcon 工作区，用于从 RViz2 交互目标生成全向移动机器人的速度指令，并提供速度指令桥接节点。

## 工作区结构

```text
ros2_interactive_marker_pid/
├── .gitignore
├── LICENSE
├── README.md
└── src/
    ├── interactive_goal_marker/
    ├── pid_goal_controller/
    └── g1_twist_bridge/
```

工作区包含三个 ROS2 包：

- `interactive_goal_marker`：在 RViz2 中提供交互式目标，并发布 `goal_pose`。
- `pid_goal_controller`：订阅目标位姿和里程计，发布 `cmd_vel`。
- `g1_twist_bridge`：订阅相对话题 `cmd_vel`，打印 Twist 的 `vx`、`vy` 和 `wz`。

当前控制链路为：

```text
interactive_goal_marker
    → goal_pose
pid_goal_controller
    → cmd_vel
g1_twist_bridge
```

`g1_twist_bridge` 当前只订阅并打印 `geometry_msgs/msg/Twist`，尚未接入 Unitree SDK2，也不包含实机运动控制代码。

各包的详细说明见：

- [`interactive_goal_marker`](src/interactive_goal_marker/README.md)
- [`pid_goal_controller`](src/pid_goal_controller/README.md)

## 构建

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 键盘测试

启动 bridge 后，可使用以下命令发布键盘速度指令：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

键盘节点默认发布 `cmd_vel`，`g1_twist_bridge` 会打印其中的 `linear.x`、`linear.y` 和 `angular.z`。
