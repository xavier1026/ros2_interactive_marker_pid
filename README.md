# Unitree G1 RViz 目标控制

本仓库是 ROS 2 Humble colcon 工作区，用 RViz 交互目标驱动 Unitree G1 高层行走控制。仓库包含三个包：

- `interactive_goal_marker`：在 `map` 中设置目标，仅在鼠标松开时发布 `/goal_pose`。
- `pid_goal_controller`：通过 TF 获取 `map → base_link`，计算位置和航向误差，发布 `/cmd_vel`。
- `g1_twist_bridge`：将 `/cmd_vel` 转换为 `unitree_api/msg/Request`，并发布到 `/api/sport/request`。

## 控制链路与坐标系

```text
RViz Interactive Marker
  → /goal_pose (frame_id: map)
PID Goal Controller
  → /cmd_vel
G1 Twist Bridge
  → /api/sport/request (unitree_api/msg/Request)
Unitree G1 high-level locomotion service
```

全局坐标系为 `map`，机器人坐标系为 `base_link`，定位系统必须发布真实的 `map → base_link` TF。本项目不依赖 `odom`，也不使用 `base_footprint`，不会为替代定位而发布假的静态 TF。RViz Fixed Frame 应设为 `map`。

## Unitree 通信

本项目不再使用 `unitree_sdk2` `ChannelFactory`，而是使用 `unitree_ros2` 提供的 `unitree_api` ROS 2 消息接口。桥接节点把 Twist 的 `linear.x`、`linear.y`、`angular.z` 映射为 `vx`、`vy`、`wz`，通过 API ID `7105` 发布 SetVelocity 请求：

```json
{"velocity":[0.100000,0.000000,0.000000],"duration":0.200000}
```

所有 ROS 2 节点统一使用 CycloneDDS，网卡由 `CYCLONEDDS_URI` 配置，不是 bridge 的 ROS 参数。

## 本地与远程环境

- 本地开发电脑用于修改代码和进行普通 ROS 2 检查，可能没有 `unitree_api`。这种情况下可只编译 `interactive_goal_marker` 和 `pid_goal_controller`。
- 远程 Orin NX 安装 ROS 2 Humble 和 `~/unitree_ros2`，用于全量编译、检查 Unitree 话题与连接实机。

本地普通包编译：

```bash
cd ~/Robot/ros2_interactive_marker_pid
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select interactive_goal_marker pid_goal_controller
```

## 环境设置

环境脚本必须用 `source`，参数是与 G1 通信的网卡：

```bash
cd ~/Robot/ros2_interactive_marker_pid
source env/g1_env.sh enP8p1s0
```

未传参时依次使用 `UNITREE_NET_IFACE` 和默认值 `enP8p1s0`。脚本仅在存在时加载 ROS 2 Humble、`~/unitree_ros2/cyclonedds_ws/install` 和本工作区的 setup 文件。

## 启动与停止

```bash
ros2 launch g1_twist_bridge g1_goal_control.launch.py \
  frame_id:=map \
  global_frame:=map \
  base_frame:=base_link \
  armed:=false \
  start_rviz:=false
```

`armed` 默认为 `false`。机器人站稳、周围清空，并确认 TF 与高层运动服务正常后，才能人工解锁：

```bash
ros2 param set /g1_twist_bridge armed true
ros2 param set /g1_twist_bridge armed false
```

第二条命令用于立即重新锁定 bridge。

## 话题与 TF 检查

```bash
ros2 topic info /goal_pose --verbose
ros2 topic info /cmd_vel --verbose
ros2 topic info /api/sport/request --verbose
ros2 topic info /api/sport/response --verbose

ros2 run tf2_ros tf2_echo map base_link
ros2 topic info /tf --verbose
ros2 topic echo /tf --once
```

## 远程 Orin NX 部署验证

更新代码：

```bash
cd ~/Robot/ros2_interactive_marker_pid
git pull --ff-only origin main
```

加载远程环境：

```bash
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
source env/g1_env.sh enP8p1s0
```

全量编译：

```bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

检查可执行文件和 launch 参数：

```bash
ros2 pkg executables | grep -E \
  'interactive_goal_marker|pid_goal_controller|g1_twist_bridge'
ros2 launch g1_twist_bridge g1_goal_control.launch.py --show-args
```

保持 bridge 锁定并启动全部节点：

```bash
ros2 launch g1_twist_bridge g1_goal_control.launch.py \
  frame_id:=map \
  global_frame:=map \
  base_frame:=base_link \
  armed:=false \
  start_rviz:=false
```

在另外的终端检查 TF 和话题：

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 topic info /goal_pose --verbose
ros2 topic info /cmd_vel --verbose
ros2 topic info /api/sport/request --verbose
ros2 topic info /api/sport/response --verbose
```

只有确认机器人安全、TF 正常、高层运动服务正常后，才允许人工执行：

```bash
ros2 param set /g1_twist_bridge armed true
```

## 安全

- 首次测试保持低速，`armed` 默认关闭。
- `/cmd_vel` 超时时 bridge 自动发送零速度；TF 丢失时 PID 立即发布零 Twist。
- 不要同时运行其他高层速度控制节点。
- 不要同时运行低层关节控制和本高层行走控制。
- 未连接真实机器人的编译或话题检查，不代表完整实机验证。

**本次改造未进行实机运动测试。**
