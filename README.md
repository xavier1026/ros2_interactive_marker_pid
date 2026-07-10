# ROS2 Interactive Marker and PID Goal Controller

面向移动机器人点到点控制的 ROS2 功能包集合，包含 RViz2 交互式目标输入和基于里程计反馈的 PID 目标控制器。

控制器输出标准 `geometry_msgs/msg/Twist` 速度指令，可接入仿真底盘，也可通过话题重映射或适配节点连接实体机器人底盘。

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

该包只负责目标输入，不参与底盘控制。

详细说明见：

[`interactive_goal_marker/README.md`](interactive_goal_marker/README.md)

### pid_goal_controller

订阅目标位姿和机器人里程计，计算机器人坐标系下的位置误差和航向误差，通过 PID 生成底盘速度指令。

输入接口：

| Topic | Message Type | Description |
|---|---|---|
| `goal_pose` | `geometry_msgs/msg/PoseStamped` | 目标位置和目标航向 |
| `odom` | `nav_msgs/msg/Odometry` | 机器人当前位姿 |

输出接口：

| Topic | Message Type | Description |
|---|---|---|
| `cmd_vel` | `geometry_msgs/msg/Twist` | 底盘线速度和角速度指令 |
| `pid_error` | `geometry_msgs/msg/TwistStamped` | 位置误差和航向误差 |
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
Robot Base
```

## Design Notes

- 两个包相互独立，话题名称均可通过参数或 ROS2 remapping 修改。
- `cmd_vel` 使用标准 `geometry_msgs/msg/Twist`，便于接入不同机器人底盘。
- PID 参数、速度上限和到达容差通过 YAML 文件配置。
