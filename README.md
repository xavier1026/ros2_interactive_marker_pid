# ROS2 Interactive Marker and PID Goal Controller

面向全向移动机器人点到点控制的 ROS2 功能包集合，包含 RViz2 交互式目标输入和基于里程计反馈的 PID 目标控制器。

控制器输出标准 `geometry_msgs/msg/Twist` 速度指令，使用 `linear.x`、`linear.y` 和 `angular.z` 三个分量，面向能够同时进行平面横移、纵移和旋转的全向底盘。普通差速底盘不能直接执行 `linear.y`，需要修改控制策略或增加适配节点。

## Package Structure

```text
ros2_interactive_marker_pid/
├── interactive_goal_marker/
└── pid_goal_controller/
```

### interactive_goal_marker

在 RViz2 中创建可平移、可旋转的 Interactive Marker，并在用户完成操作后发布目标位姿。

输出接口：

| Topic | Message Type | Description |
|---|---|---|
| `goal_pose` | `geometry_msgs/msg/PoseStamped` | 目标位置和目标航向 |

该包只负责目标输入，不参与底盘控制，也不发布 `cmd_vel`。

详细说明见：

[`interactive_goal_marker/README.md`](interactive_goal_marker/README.md)

### pid_goal_controller

订阅目标位姿和机器人里程计，计算机器人机体坐标系下的位置误差和航向误差，通过三个 PID 通道生成全向底盘速度指令。

输入接口：

| Topic | Message Type | Description |
|---|---|---|
| `goal_pose` | `geometry_msgs/msg/PoseStamped` | 目标位置和目标航向 |
| `odom` | `nav_msgs/msg/Odometry` | 机器人当前位姿 |

输出接口：

| Topic | Message Type | Description |
|---|---|---|
| `cmd_vel` | `geometry_msgs/msg/Twist` | 底盘线速度和角速度指令 |
| `pid_error` | `geometry_msgs/msg/TwistStamped` | 位置误差和航向误差调试信息 |
| `pid_distance` | `std_msgs/msg/Float64` | 机器人与目标之间的平面距离 |

详细说明见：

[`pid_goal_controller/README.md`](pid_goal_controller/README.md)

## Data Flow

```text
RViz2 Interactive Marker
          |
          | goal_pose
          v
PID Goal Controller <----- odom
          |
          | cmd_vel
          v
Omnidirectional Robot Base
```

## Coordinate Frames

控制器当前不会自动查询 TF，也不会进行坐标转换。`goal_pose.header.frame_id` 应与里程计位姿使用的坐标系一致。

默认交互标记和里程计均按 `odom` 坐标系使用。如果目标来自 `map` 等其他坐标系，需要先完成坐标转换，再发送给控制器。

## Design Notes

- 两个包相互独立，话题名称均可通过参数或 ROS2 remapping 修改。
- `interactive_goal_marker` 只发布目标位姿。
- `pid_goal_controller` 输出 `linear.x`、`linear.y` 和 `angular.z`，适用于全向移动底盘。
- PID 参数、速度上限和到达容差通过 YAML 文件配置。
