# pid_goal_controller

`pid_goal_controller` 根据目标位姿和里程计反馈，为全向移动底盘生成平面速度指令。

## 功能说明

- 订阅目标位姿和里程计。
- 计算世界坐标系位置误差。
- 将位置误差转换到机器人机体坐标系。
- 使用三个 PID 通道分别生成机体坐标系的 `x`、`y` 和航向控制量。
- 输出全向底盘速度指令。
- 到达目标后输出零速度。
- 里程计超时后停止机器人。

## 接口

订阅：

| 名称 | 消息类型 | 说明 |
|---|---|---|
| `goal_pose` | `geometry_msgs/msg/PoseStamped` | 目标位置与目标航向 |
| `odom` | `nav_msgs/msg/Odometry` | 机器人当前位姿 |

发布：

| 名称 | 消息类型 | 说明 |
|---|---|---|
| `cmd_vel` | `geometry_msgs/msg/Twist` | 全向底盘速度指令 |
| `pid_error` | `geometry_msgs/msg/TwistStamped` | PID 调试误差 |
| `pid_distance` | `std_msgs/msg/Float64` | 机器人与目标之间的平面距离 |

`pid_error` 是调试话题，不是速度指令：

```text
pid_error.twist.linear.x  = 机体坐标系x方向位置误差
pid_error.twist.linear.y  = 机体坐标系y方向位置误差
pid_error.twist.angular.z = 航向角误差
```

`pid_distance.data` 表示世界坐标系下机器人当前位置与目标位置之间的平面距离。

## 适用底盘

该控制器默认面向全向移动底盘，因为会同时输出 `linear.x`、`linear.y` 和 `angular.z`。
普通差速底盘不能直接执行 `linear.y`，需要修改控制策略或增加适配节点。

## 坐标系限制

- 当前节点不查询 TF。
- 目标位姿坐标系必须与里程计位姿坐标系一致。
- 默认推荐使用 `odom`。
- 如果目标来自 `map`，应先转换到 `odom`，或修改节点加入 TF 转换。

## 参数

话题名称参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `goal_topic` | `goal_pose` | 订阅目标位姿的 Topic |
| `odom_topic` | `odom` | 订阅里程计的 Topic |
| `cmd_vel_topic` | `cmd_vel` | 发布速度指令的 Topic |
| `error_topic` | `pid_error` | 发布误差调试信息的 Topic |
| `distance_topic` | `pid_distance` | 发布平面距离的 Topic |

PID 参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `kp_x` | `1.0` | 机体坐标系 x 方向比例系数 |
| `ki_x` | `0.0` | 机体坐标系 x 方向积分系数 |
| `kd_x` | `0.05` | 机体坐标系 x 方向微分系数 |
| `kp_y` | `1.0` | 机体坐标系 y 方向比例系数 |
| `ki_y` | `0.0` | 机体坐标系 y 方向积分系数 |
| `kd_y` | `0.05` | 机体坐标系 y 方向微分系数 |
| `kp_yaw` | `2.0` | 航向比例系数 |
| `ki_yaw` | `0.0` | 航向积分系数 |
| `kd_yaw` | `0.08` | 航向微分系数 |

控制参数：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `max_v` | `0.4` | 最大平面线速度 |
| `max_w` | `1.0` | 最大角速度 |
| `pos_tolerance` | `0.03` | 位置到达容差 |
| `yaw_tolerance` | `0.05` | 航向到达容差 |
| `control_frequency` | `30.0` | 控制循环频率 |
| `odom_timeout` | `0.5` | 里程计超时时间 |
| `use_sim_time` | `false` | 是否使用仿真时间 |

## 启动命令

```bash
ros2 launch pid_goal_controller pid_goal_controller.launch.py
```
