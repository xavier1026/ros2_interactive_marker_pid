# interactive_goal_marker

`interactive_goal_marker` 在 RViz2 中创建 Interactive Marker，用于输入机器人目标位姿。

## 功能说明

- 在 RViz2 中创建 Interactive Marker。
- 支持平面拖动目标位置。
- 支持旋转目标航向。
- 用户完成操作并释放鼠标后发布目标位姿。
- 只负责目标输入，不负责 PID 控制，也不发布 `/cmd_vel`。

## 接口

| 类型 | 名称 | 消息类型 | 说明 |
|---|---|---|---|
| 发布话题 | `/goal_pose` | `geometry_msgs/msg/PoseStamped` | `map` 中的目标位置与目标航向 |

## 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `frame_id` | `map` | Interactive Marker 和发布目标位姿使用的坐标系（仅支持 `map`） |
| `server_name` | `goal_marker` | Interactive Marker Server 名称 |
| `goal_topic` | `/goal_pose` | 发布目标位姿的 Topic |
| `use_sim_time` | `false` | 是否使用仿真时间 |

## 启动命令

```bash
ros2 launch interactive_goal_marker interactive_goal_marker.launch.py
```

## RViz2 配置

1. 打开 RViz2。
2. 将 `Fixed Frame` 设置为 `map`。
3. 点击 `Add`。
4. 添加 `InteractiveMarkers` 显示项。
5. 将 `Update Topic` 设置为或选择 `/goal_marker/update`。
6. 拖动标记调整目标位置，使用旋转控件调整目标朝向。
7. 拖动期间只更新标记显示；释放鼠标后，节点向 `/goal_pose` 发布一次 `PoseStamped`，其 `header.frame_id` 为 `map`。

`/goal_marker/update` 来源于默认的 `server_name=goal_marker`；修改 `server_name` 后，对应的更新话题也会变化。
