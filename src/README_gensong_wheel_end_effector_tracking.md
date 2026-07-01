# Gensong 两段式抓取/放置启动说明

本文档说明当前 `gensong_board` 双手末端两段式搬运流程怎么启动、怎么手动发 topic、以及每一步会发生什么。

当前流程已经不是旧版的“一次把抓取到放置整段轨迹都规划出来”，而是两段式：

1. 发送 `box_pose`，机器人自动执行第一段：抓取 -> 抬起 -> 持箱等待。
2. 发送 `place_box_pose`，机器人自动执行第二段：搬运 -> 预放置 -> 放置 -> 后撤。

在这两段之间，双手和箱子保持刚体关系，不会中途松掉箱子。

## 1. 当前接口语义

- `box_pose`
  - 类型：`geometry_msgs/msg/PoseStamped`
  - 语义：箱子当前中心位姿
  - 触发：当规划器处于 `IDLE` 时，收到它会自动启动第一段抓取

- `place_box_pose`
  - 类型：`geometry_msgs/msg/PoseStamped`
  - 语义：箱子最终放置中心位姿
  - 触发：当规划器处于 `HOLDING_OBJECT` 时，收到它会自动启动第二段放置

- `plan_and_send_grasp_trajectory`
  - 类型：`std_srvs/srv/Trigger`
  - 作用：兼容保留，只用于“基于最近一次 `box_pose` 手动触发第一段”
  - 现在不是主流程必需接口

注意：

- `box_pose.header.frame_id` 和 `place_box_pose.header.frame_id` 都必须是 `base_link`，或者你当前配置的 `planning_frame`
- `place_box_pose` 在机器人还没抓住箱子前发送，会被忽略
- `box_pose` 在机器人已经抓住箱子后再次发送，也会被忽略

## 2. 环境准备

在工作区根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

如果你刚改过代码、launch 或 URDF，建议重新编译：

```bash
colcon build --packages-select ocs2_robotic_assets ocs2_mobile_manipulator ocs2_mobile_manipulator_ros
source install/setup.bash
```

## 3. 最快的测试启动方式

如果你只是想快速验证“先抓取，再等放置点，再放置”的完整链路，直接启动：

```bash
ros2 launch ocs2_mobile_manipulator_ros grasp_waypoint_test.launch.py
```

无图形环境可以用：

```bash
ros2 launch ocs2_mobile_manipulator_ros grasp_waypoint_test.launch.py rviz:=false
```

这个 launch 会同时启动：

- 机器人本体：`manipulator_gensong_dual.launch.py`
- 两段式规划器：`dual_arm_grasp_waypoint_planner`
- 预设测试发布器：`preset_dual_arm_grasp_pose_publisher.py`

默认行为：

1. 先持续发布 `box_pose`
2. 机器人自动执行抓取并抬起
3. 默认 `8` 秒后开始发布 `place_box_pose`
4. 机器人自动继续放置

如果你想改成更长等待时间：

```bash
ros2 launch ocs2_mobile_manipulator_ros grasp_waypoint_test.launch.py place_trigger_delay_sec:=12.0
```

如果你想改“抓到后先停留再抬起”的时间：

```bash
ros2 launch ocs2_mobile_manipulator_ros grasp_waypoint_planner.launch.py grasp_hold_sec:=2.0
```

如果你的识别结果是箱子中心点，但实际抓取点要在“中心点上方 1cm”的位置：

```bash
ros2 launch ocs2_mobile_manipulator_ros grasp_waypoint_planner.launch.py
```

如果你想把整段抓取/放置动作都放慢一点，可以把统一时间倍率调大，比如 `1.5`：

```bash
ros2 launch ocs2_mobile_manipulator_ros grasp_waypoint_planner.launch.py trajectory_time_scale:=1.5
```

如果你用的是 `grasp_waypoint_test.launch.py`，记得把 `place_trigger_delay_sec` 也相应调大，不然它可能在动作还没结束时就提前发放置点。

## 4. 手动启动真实流程

如果 `box_pose` 和 `place_box_pose` 是你自己或外部节点发的，推荐手动分两个终端启动。

### 4.1 启动机器人本体

终端 1：

```bash
source /opt/ros/jazzy/setup.bash
source /home/robot-zhao/work/gensong_ros2/install/setup.bash
ros2 launch ocs2_mobile_manipulator_ros manipulator_gensong_dual.launch.py
```

无图形环境：

```bash
source /opt/ros/jazzy/setup.bash
source /home/robot-zhao/work/gensong_ros2/install/setup.bash
ros2 launch ocs2_mobile_manipulator_ros manipulator_gensong_dual.launch.py rviz:=false
```

### 4.2 启动两段式规划器

终端 2：

```bash
source /opt/ros/jazzy/setup.bash
source /home/robot-zhao/work/gensong_ros2/install/setup.bash
ros2 launch ocs2_mobile_manipulator_ros grasp_waypoint_planner.launch.py
```

如果你的 topic 名不是默认值，可以启动时改掉：

```bash
ros2 launch ocs2_mobile_manipulator_ros grasp_waypoint_planner.launch.py \
  box_pose_topic:=/my_box_pose \
  place_box_pose_topic:=/my_place_box_pose
```

## 5. 手动发布抓取点和放置点

### 5.1 发布抓取点 `box_pose`

终端 3：

```bash
source /opt/ros/jazzy/setup.bash
source /home/robot-zhao/work/gensong_ros2/install/setup.bash
ros2 topic pub --once /box_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: base_link}, pose: {position: {x: 0.8994, y: -0.0327, z: 1.1000}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

发送后，规划器会自动启动第一段，不需要再调 service。

第一段结束后，机器人会停在“抬起后的持箱等待位”。

### 5.2 发布放置点 `place_box_pose`

确认机器人已经抓起并稳定等待后，再发送放置点。

终端 4：

```bash
source /opt/ros/jazzy/setup.bash
source /home/robot-zhao/work/gensong_ros2/install/setup.bash
ros2 topic pub --once /place_box_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: base_link}, pose: {position: {x: 0.7500, y: 0.2000, z: 1.1000}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

发送后，规划器会自动启动第二段放置。

### 5.3 如果你还想手动用旧 service 触发第一段

只有在下面两个条件同时满足时才有意义：

- 已经发过一次 `box_pose`
- 当前状态仍然是 `IDLE`

命令：

```bash
ros2 service call /plan_and_send_grasp_trajectory std_srvs/srv/Trigger "{}"
```

主流程里一般不需要这个命令。

## 6. 一套完整的手动测试顺序

推荐按下面顺序验证：

### 终端 1

```bash
source /opt/ros/jazzy/setup.bash
source /home/robot-zhao/work/gensong_ros2/install/setup.bash
ros2 launch ocs2_mobile_manipulator_ros manipulator_gensong_dual.launch.py
```

### 终端 2

```bash
source /opt/ros/jazzy/setup.bash
source /home/robot-zhao/work/gensong_ros2/install/setup.bash
ros2 launch ocs2_mobile_manipulator_ros grasp_waypoint_planner.launch.py
```

### 终端 3，发抓取点

```bash
source /opt/ros/jazzy/setup.bash
source /home/robot-zhao/work/gensong_ros2/install/setup.bash
ros2 topic pub --once /box_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: base_link}, pose: {position: {x: 0.8994, y: -0.0327, z: 1.1000}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

等待机器人完成抓取并抬起。

### 终端 4，发放置点

```bash
source /opt/ros/jazzy/setup.bash
source /home/robot-zhao/work/gensong_ros2/install/setup.bash
ros2 topic pub --once /place_box_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: base_link}, pose: {position: {x: 0.7500, y: 0.2000, z: 1.1000}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

## 7. 关键行为说明

- `box_pose` 是抓取输入，不是放置输入
- `place_box_pose` 必须在第一段结束后再发
- 第二段放置时，箱子姿态会跟随 `place_box_pose.orientation`
- 旧的 `enable_transport_stage`、`enable_place_stage`、`transport_offset_*` 参数仍然保留在 launch 里，但对当前自动两段式流程已经不再是主控制接口

特别注意：

- 如果你的外部节点持续不断发布新的 `box_pose`，那么机器人在一次放置完成回到 `IDLE` 后，后续新消息可能再次触发下一轮抓取
- 最稳妥的方式是：抓取阶段只发一次 `box_pose`，放置阶段只发一次 `place_box_pose`

## 8. 当前默认配置

- 机器人 URDF：`src/ocs2_robotic_assets/resources/gensong_board/urdf/gensong_board.urdf`
- 双末端帧：`handboard_left`、`handboard_right`
- 任务文件：`src/ocs2/ocs2_robotic_examples/ocs2_mobile_manipulator/config/gensong/task_dual_ee.info`
- 默认箱子尺寸：`box_size_x=0.1978`、`box_size_y=0.2966`、`box_size_z=0.1464`（单位：米，对应 `19.78 x 29.66 x 14.64` 厘米）
- 默认 topic：
  - `/box_pose`
  - `/place_box_pose`

## 9. 常见问题

### 9.1 发了 `box_pose` 没反应

优先检查：

- `dual_arm_grasp_waypoint_planner` 是否已经启动
- 是否已经收到 `mobile_manipulator_mpc_observation`
- `box_pose.header.frame_id` 是否为 `base_link`
- 当前状态是不是已经不是 `IDLE`

可用：

```bash
ros2 topic echo /box_pose
```

### 9.2 发了 `place_box_pose` 没反应

通常是因为当前还没进入 `HOLDING_OBJECT`。

也就是：

- 机器人还没完成第一段抓取
- 或者第一段根本没成功启动

可用：

```bash
ros2 topic echo /place_box_pose
```

### 9.3 为什么还看到 `plan_and_send_grasp_trajectory`

这是兼容保留接口。

当前主流程是：

- `box_pose` 自动触发抓取
- `place_box_pose` 自动触发放置

不是必须靠 service 才能规划。

### 9.4 Mesh 或 URDF 找不到

重新编译资源包：

```bash
colcon build --packages-select ocs2_robotic_assets ocs2_mobile_manipulator_ros
source install/setup.bash
```

如果想确认文件存在，可检查：

```bash
ls /home/robot-zhao/work/gensong_ros2/src/ocs2_robotic_assets/resources/gensong_board/urdf/gensong_board.urdf
```
