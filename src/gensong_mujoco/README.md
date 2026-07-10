# gensong_mujoco

This package runs the Gensong MuJoCo scene with ROS 2 interfaces.

Install the Python runtime dependency in the same environment used by ROS 2:

```bash
python3 -m pip install mujoco
```

After building and sourcing the workspace:

```bash
ros2 launch gensong_mujoco gensong_mujoco.launch.py
```

The interactive MuJoCo window is enabled by default. For headless operation,
use `ros2 launch gensong_mujoco gensong_mujoco.launch.py viewer:=false`.

Interfaces:

- Subscribe to `/gensong/joint_command` (`sensor_msgs/msg/JointState`); named
  messages are matched by joint name, unnamed messages follow model order.
- Subscribe to `/cmd_vel` (`geometry_msgs/msg/Twist`) for planar base velocity.
- Publish `/gensong/joint_states` (`sensor_msgs/msg/JointState`).
- The simulation and joint-state publisher default to 500 Hz. The MuJoCo
  bridge reads the `scene_box` center from the configured XML at startup and
  uses bounded position/velocity feedback (`position_gain`, `velocity_gain`,
  `torque_limit`) to reduce contact oscillation.
- Publish RGB-D on `/gensong/camera/{color,depth}/image_raw` and matching
  `camera_info` topics.
