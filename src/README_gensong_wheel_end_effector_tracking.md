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

## 4. 默认配置说明

- 机器人：`gensong_wheel_outfit.urdf`
- 末端帧：`wrist3_left_link`、`wrist3_right_link`
- 模式：固定基座（`manipulatorModelType = 0`）
- 任务文件：`task_dual_ee.info`

## 5. 常见问题

### 5.1 Mesh 加载失败（`Could not load mesh resource`）

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

### 5.2 URDF 文件不存在（`URDF file not found`）

处理：

```bash
ros2 pkg prefix ocs2_robotic_assets
ls install/ocs2_robotic_assets/share/ocs2_robotic_assets/resources/gensong_wheel_outfit_cover/urdf/gensong_wheel_outfit.urdf
```

如果文件不存在，重新执行资源包编译安装即可。

