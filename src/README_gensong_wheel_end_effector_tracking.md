# Gensong Wheel 双末端追踪使用说明

本文档说明如何启动 `gensong_wheel` 的双末端 6D 追踪（左右手末端）。

## 1. 环境准备

在工作区根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

如果你刚改过 URDF/mesh，建议先重新编译并安装资源包：

```bash
colcon build --packages-select ocs2_robotic_assets ocs2_mobile_manipulator ocs2_mobile_manipulator_ros
source install/setup.bash
```

如果你刚改过 `launch`、`rviz` 或 `ocs2_mobile_manipulator_ros` 里的代码，也要重新编译并重新执行一次：

```bash
colcon build --packages-select ocs2_mobile_manipulator_ros
source install/setup.bash
```

## 2. 启动双末端追踪

```bash
ros2 launch ocs2_mobile_manipulator_ros manipulator_gensong_dual.launch.py
```

无图形环境（只验证节点是否正常）可用：

```bash
ros2 launch ocs2_mobile_manipulator_ros manipulator_gensong_dual.launch.py rviz:=false
```

## 3. RViz 中如何发送末端目标

启动后会出现两个交互 Marker：

- `left_goal`
- `right_goal`

操作方式：

1. 拖动某个 Marker 到目标位姿（位置 + 朝向）。
2. 右键该 Marker，点击 `Send target pose`。
3. 只发送左手时，右手保持上一次目标；只发送右手同理。

## 4. 启动箱子中心位姿抓取测试

如果你要测试“只订阅箱子中心位姿 `box_pose`，左右手抓取点由箱体尺寸自动计算”的新流程，执行：

```bash
ros2 launch ocs2_mobile_manipulator_ros grasp_waypoint_test.launch.py
```

无图形环境可用：

```bash
ros2 launch ocs2_mobile_manipulator_ros grasp_waypoint_test.launch.py rviz:=false
```

这个 launch 会同时启动：

- `manipulator_gensong_dual.launch.py`
- `dual_arm_grasp_waypoint_planner`
- `preset_dual_arm_grasp_pose_publisher.py`

其中规划器只订阅 `box_pose`，箱体尺寸通过 launch 参数传入，左右手抓取位姿在节点内部自动计算。

## 5. 如果外部节点已经在发布 `box_pose`

如果 `box_pose` 由别人提供，你只需要启动抓取规划器：

```bash
ros2 launch ocs2_mobile_manipulator_ros grasp_waypoint_planner.launch.py
```

如果对方发的不是默认 topic 名 `box_pose`，可以改成一致：

```bash
ros2 launch ocs2_mobile_manipulator_ros grasp_waypoint_planner.launch.py box_pose_topic:=/your_box_pose_topic
```

注意：

- 对方发布的消息类型必须是 `geometry_msgs/msg/PoseStamped`
- `header.frame_id` 默认需要是 `base_link`

如果你还需要同时启动机器人本体和 RViz，可再开一个终端执行：

```bash
ros2 launch ocs2_mobile_manipulator_ros manipulator_gensong_dual.launch.py
```

## 6. 默认配置说明

- 机器人：`gensong_wheel_outfit.urdf`
- 末端帧：`wrist3_left_link`、`wrist3_right_link`
- 模式：固定基座（`manipulatorModelType = 0`）
- 任务文件：`task_dual_ee.info`

## 7. 常见问题

### 7.1 Mesh 加载失败（`Could not load mesh resource`）

现象示例：

- `package://gsrobot_description/meshes/...` 找不到

原因：

- URDF 指向了不存在的包路径。

处理：

1. 确认 URDF 使用的是 `ocs2_robotic_assets` 下的 mesh 路径。
2. 重新安装资源包：

```bash
colcon build --packages-select ocs2_robotic_assets
source install/setup.bash
```

### 7.2 URDF 文件不存在（`URDF file not found`）

处理：

```bash
ros2 pkg prefix ocs2_robotic_assets
ls install/ocs2_robotic_assets/share/ocs2_robotic_assets/resources/gensong_wheel_outfit_cover/urdf/gensong_wheel_outfit.urdf
```

如果文件不存在，重新执行资源包编译安装即可。
