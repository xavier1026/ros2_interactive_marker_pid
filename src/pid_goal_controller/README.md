# pid_goal_controller

`pid_goal_controller` 根据全局目标位姿和实时 TF，为全向移动底盘生成平面速度指令。

本项目以 `map` 作为全局坐标系，以 `base_link` 作为机器人本体坐标系。PID 控制器通过 TF 查询 `map → base_link`，获得机器人在地图中的实时位姿。

## 控制流程

控制器接收 `map` 中的目标位姿，并在每个控制周期调用：

```cpp
lookupTransform("map", "base_link", tf2::TimePointZero);
```

该查询取得 `base_link` 在 `map` 中的当前位置和朝向。控制器先计算目标与机器人之间的 `map` 坐标系误差，再用机器人当前航向将平移误差旋转到 `base_link`，最后由三个 PID 通道分别生成 `linear.x`、`linear.y` 和 `angular.z`。

目标消息的 `frame_id` 为 `map` 时直接使用。其他坐标系的目标必须先通过 tf2 转换到 `map`；转换失败或 `frame_id` 为空时，控制器会拒绝目标、清除此前目标并立即发布零速度，不会通过改写 `frame_id` 伪造转换。

当 `map → base_link` TF 不可用时，每个控制周期都会立即发布零 `Twist`，重置 PID，并通过节流日志报告错误。控制器不会复用旧位姿或旧速度；真实定位 TF 恢复后才会继续控制。本包不会发布替代定位系统的静态 TF。

## 接口

订阅：

| 名称 | 消息类型 | 说明 |
|---|---|---|
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | 默认位于 `map` 的目标位置与航向 |

发布：

| 名称 | 消息类型 | 说明 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `base_link` 坐标系下的全向底盘速度指令 |
| `/pid_error` | `geometry_msgs/msg/TwistStamped` | `base_link` 坐标系下的 PID 调试误差 |
| `/pid_distance` | `std_msgs/msg/Float64` | `map` 中机器人与目标的平面距离 |

该控制器默认面向能执行 `linear.x`、`linear.y` 和 `angular.z` 的全向移动底盘。

## 参数

坐标系与话题参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `global_frame` | `map` | 全局定位与目标坐标系 |
| `base_frame` | `base_link` | 机器人本体坐标系 |
| `goal_topic` | `/goal_pose` | 目标位姿话题 |
| `cmd_vel_topic` | `/cmd_vel` | 速度指令话题 |
| `error_topic` | `/pid_error` | 误差调试话题 |
| `distance_topic` | `/pid_distance` | 平面距离调试话题 |

PID 参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `kp_x`, `ki_x`, `kd_x` | `1.0`, `0.0`, `0.05` | 机体 x 方向 PID |
| `kp_y`, `ki_y`, `kd_y` | `1.0`, `0.0`, `0.05` | 机体 y 方向 PID |
| `kp_yaw`, `ki_yaw`, `kd_yaw` | `2.0`, `0.0`, `0.08` | 航向 PID |

控制参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `max_v` | `0.10` | 最大平面线速度（m/s） |
| `max_w` | `0.15` | 最大角速度（rad/s） |
| `pos_tolerance` | `0.10` | 位置到达容差（m） |
| `yaw_tolerance` | `0.10` | 航向到达容差（rad） |
| `control_frequency` | `30.0` | 控制循环频率 |
| `transform_timeout` | `0.1` | 单次 TF 查询超时（秒） |
| `publish_debug` | `true` | 是否发布误差和距离调试话题 |

## 启动与 TF 检查

```bash
ros2 launch pid_goal_controller pid_goal_controller.launch.py \
  global_frame:=map \
  base_frame:=base_link
```

定位系统必须提供所需 TF，可用以下命令检查：

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 topic info /tf --verbose
ros2 topic echo /tf --once
```
