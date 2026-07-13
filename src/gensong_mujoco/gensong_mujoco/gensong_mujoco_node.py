#!/usr/bin/python3
"""ROS 2 bridge for the Gensong MuJoCo scene."""

import threading
import time
import os
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Bool

try:
    from gensong_mujoco.box_lift_policy import load_box_center
except ModuleNotFoundError:  # installed ROS executable shares its directory with the helper
    from box_lift_policy import load_box_center

try:
    # This node publishes off-screen camera frames and does not create a GUI.
    # EGL avoids X11/GLX context failures on remote displays and headless hosts.
    os.environ.setdefault("MUJOCO_GL", "egl")
    import mujoco
    import mujoco.viewer
except ImportError as exc:  # pragma: no cover - exercised only on missing runtime dependency
    mujoco = None
    _MUJOCO_IMPORT_ERROR = exc


class GensongMujoco(Node):
    def __init__(self):
        super().__init__("gensong_mujoco")
        if mujoco is None:
            raise RuntimeError("Python package 'mujoco' is required") from _MUJOCO_IMPORT_ERROR

        default_model = Path(get_package_share_directory("ocs2_robotic_assets")) / \
            "resources/gensong_board/mjcf/gensong_scene.xml"
        self.declare_parameter("model_path", str(default_model))
        self.declare_parameter("simulation_hz", 500.0)
        self.declare_parameter("camera_hz", 30.0)
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("color_topic", "/gensong/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/gensong/camera/depth/image_raw")
        self.declare_parameter("color_info_topic", "/gensong/camera/color/camera_info")
        self.declare_parameter("depth_info_topic", "/gensong/camera/depth/camera_info")
        self.declare_parameter("box_pose_topic", "/box_pose")
        self.declare_parameter("box_state_topic", "/gensong/box_pose")
        self.declare_parameter("box_grasped_topic", "/gensong/box_grasped")
        self.declare_parameter("viewer", True)
        self.declare_parameter("position_gain", 1200.0)
        self.declare_parameter("velocity_gain", 120.0)
        self.declare_parameter("leg_position_gain", 8000.0)
        self.declare_parameter("leg_velocity_gain", 400.0)
        self.declare_parameter("waist_position_gain", 5000.0)
        self.declare_parameter("waist_velocity_gain", 300.0)
        self.declare_parameter("arm_position_gain", 4000.0)
        self.declare_parameter("arm_velocity_gain", 250.0)
        self.declare_parameter("wrist_position_gain", 1500.0)
        self.declare_parameter("wrist_velocity_gain", 120.0)
        self.declare_parameter("torque_limit", 300.0)
        self.declare_parameter("joint_state_publish_hz", 500.0)

        self.model = mujoco.MjModel.from_xml_path(self.get_parameter("model_path").value)
        self.data = mujoco.MjData(self.model)
        self.box_center = load_box_center(self.get_parameter("model_path").value)
        self.get_logger().info(
            "Loaded scene_box center: [%.4f, %.4f, %.4f]" % tuple(self.box_center)
        )
        self.lock = threading.Lock()
        self.running = True
        self.box_body_id = self.model.body("scene_box").id
        self.box_qpos_addr = self.model.joint("scene_box_freejoint").qposadr[0]
        self.box_qvel_addr = self.model.joint("scene_box_freejoint").dofadr[0]
        self.left_hand_geom_id = self.model.geom("handboard_left_collision").id
        self.right_hand_geom_id = self.model.geom("handboard_right_collision").id
        self.box_collision_geom_id = self.model.geom("scene_box_collision").id
        self.box_initial_z = float(self.data.xpos[self.box_body_id][2])
        self.box_contacting = False
        self.box_grasped = False
        self._next_contact_report = 0.0
        self.command = np.zeros(self.model.nv)
        self.base_command = np.zeros(3)
        self.control_names = [
            self.model.joint(i).name for i in range(self.model.njnt)
            if self.model.joint(i).name not in {"base_x", "base_y", "base_yaw", "scene_box_freejoint"}
            and self.model.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE
        ]
        self.qpos_addr = {n: self.model.joint(n).qposadr[0] for n in self.control_names}
        self.qvel_addr = {n: self.model.joint(n).dofadr[0] for n in self.control_names}
        self.actuator_ids = {}
        for actuator_id in range(self.model.nu):
            joint_id = int(self.model.actuator_trnid[actuator_id, 0])
            joint_name = self.model.joint(joint_id).name
            self.actuator_ids.setdefault(joint_name, []).append(actuator_id)
        self.base_qvel = {
            "x": self.model.joint("base_x").dofadr[0],
            "y": self.model.joint("base_y").dofadr[0],
            "yaw": self.model.joint("base_yaw").dofadr[0],
        }

        self.state_pub = self.create_publisher(JointState, "/gensong/joint_states", 10)
        self.color_pub = self.create_publisher(Image, self.get_parameter("color_topic").value, 10)
        self.depth_pub = self.create_publisher(Image, self.get_parameter("depth_topic").value, 10)
        self.color_info_pub = self.create_publisher(CameraInfo, self.get_parameter("color_info_topic").value, 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, self.get_parameter("depth_info_topic").value, 10)
        self.box_state_pub = self.create_publisher(PoseStamped, self.get_parameter("box_state_topic").value, 10)
        self.box_grasped_pub = self.create_publisher(Bool, self.get_parameter("box_grasped_topic").value, 10)
        self.create_subscription(
            PoseStamped, self.get_parameter("box_pose_topic").value, self.on_box_pose, 10
        )
        self.create_subscription(JointState, "/gensong/joint_command", self.on_joint_command, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)

        self.sim_thread = threading.Thread(target=self.simulation_loop, daemon=True)
        self.camera_thread = threading.Thread(target=self.camera_loop, daemon=True)
        self.sim_thread.start()
        self.camera_thread.start()

    def on_joint_command(self, msg):
        with self.lock:
            if msg.name:
                for name, value in zip(msg.name, msg.position):
                    if name in self.qpos_addr:
                        self.command[self.qpos_addr[name]] = value
            else:
                for name, value in zip(self.control_names, msg.position):
                    self.command[self.qpos_addr[name]] = value

    def on_box_pose(self, msg):
        """Set the physical free body's initial pose to the planner input."""
        if msg.header.frame_id and msg.header.frame_id != "base_link":
            self.get_logger().warn(
                "Ignoring box_pose in frame '%s'; expected 'base_link'." % msg.header.frame_id
            )
            return
        quaternion = np.array(
            [msg.pose.orientation.w, msg.pose.orientation.x,
             msg.pose.orientation.y, msg.pose.orientation.z], dtype=float
        )
        norm = np.linalg.norm(quaternion)
        if norm < 1e-9:
            self.get_logger().warn("Ignoring box_pose with a zero quaternion.")
            return
        quaternion /= norm
        position = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float
        )
        with self.lock:
            self.data.qpos[self.box_qpos_addr:self.box_qpos_addr + 3] = position
            self.data.qpos[self.box_qpos_addr + 3:self.box_qpos_addr + 7] = quaternion
            self.data.qvel[self.box_qvel_addr:self.box_qvel_addr + 6] = 0.0
            self.box_initial_z = float(position[2])
            self.box_contacting = False
            self.box_grasped = False
            mujoco.mj_forward(self.model, self.data)
        self.get_logger().info(
            "Synchronized MuJoCo scene_box to box_pose: [%.4f, %.4f, %.4f]"
            % tuple(position)
        )
    def on_cmd_vel(self, msg):
        with self.lock:
            self.base_command[:] = (msg.linear.x, msg.linear.y, msg.angular.z)

    def _has_dual_hand_contact(self):
        contacts = set()
        force_by_hand = {"left": 0.0, "right": 0.0}
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            pair = {int(contact.geom1), int(contact.geom2)}
            if self.box_collision_geom_id in pair:
                force = np.zeros(6)
                mujoco.mj_contactForce(self.model, self.data, index, force)
                if self.left_hand_geom_id in pair:
                    force_by_hand["left"] = max(force_by_hand["left"], float(force[0]))
                    if force[0] > 0.5:
                        contacts.add("left")
                if self.right_hand_geom_id in pair:
                    force_by_hand["right"] = max(force_by_hand["right"], float(force[0]))
                    if force[0] > 0.5:
                        contacts.add("right")
        now = time.monotonic()
        if now >= self._next_contact_report:
            self._next_contact_report = now + 0.25
            box_z = float(self.data.xpos[self.box_body_id][2])
            left_pos = self.data.geom_xpos[self.left_hand_geom_id]
            right_pos = self.data.geom_xpos[self.right_hand_geom_id]
            self.get_logger().info(
                "MuJoCo contact diagnostics: z=%.4f, ncon=%d, "
                "left_force=%.3f right_force=%.3f, "
                "left=[%.3f %.3f %.3f] right=[%.3f %.3f %.3f]"
                % (
                    box_z,
                    self.data.ncon,
                    force_by_hand["left"],
                    force_by_hand["right"],
                    left_pos[0], left_pos[1], left_pos[2],
                    right_pos[0], right_pos[1], right_pos[2],
                )
            )
        return contacts == {"left", "right"}

    def _update_physical_grasp_state(self):
        contacting = self._has_dual_hand_contact()
        box_z = float(self.data.xpos[self.box_body_id][2])
        box_speed = float(np.linalg.norm(self.data.qvel[self.box_qvel_addr + 3:self.box_qvel_addr + 6]))
        # Require a meaningful clearance above the table-settled pose.  The
        # wider speed margin prevents contact-solver transients from making a
        # physically held box appear to alternate between grasped/released.
        grasped = contacting and box_z >= self.box_initial_z + 0.05 and box_speed <= 0.35
        if contacting != self.box_contacting:
            self.get_logger().info("MuJoCo bilateral physical contact: %s" % contacting)
        if grasped != self.box_grasped:
            self.get_logger().info(
                "MuJoCo physical grasp state: %s (z=%.4f, speed=%.4f)" % (grasped, box_z, box_speed)
            )
        self.box_contacting = contacting
        self.box_grasped = grasped

    def simulation_loop(self):
        simulation_hz = float(self.get_parameter("simulation_hz").value)
        state_hz = float(self.get_parameter("joint_state_publish_hz").value)
        state_interval = max(1, round(simulation_hz / state_hz))
        period = 1.0 / simulation_hz
        next_tick = time.monotonic()
        step_count = 0
        while self.running and rclpy.ok():
            with self.lock:
                for name, addr in self.qpos_addr.items():
                    error = self.command[addr] - self.data.qpos[addr]
                    velocity = self.data.qvel[self.qvel_addr[name]]
                    if name in {"leg_low_joint", "leg_up_joint"}:
                        gain_prefix = "leg"
                    elif name in {"waist_low_joint", "waist_up_joint", "chest_joint"}:
                        gain_prefix = "waist"
                    elif "wrist" in name:
                        gain_prefix = "wrist"
                    elif "arm" in name:
                        gain_prefix = "arm"
                    else:
                        gain_prefix = "default"
                    position_gain = float(self.get_parameter(
                        f"{gain_prefix}_position_gain" if gain_prefix != "default" else "position_gain").value)
                    velocity_gain = float(self.get_parameter(
                        f"{gain_prefix}_velocity_gain" if gain_prefix != "default" else "velocity_gain").value)
                    # Hold the model's zero-angle initial pose against gravity.
                    torque = (
                        position_gain * error - velocity_gain * velocity
                    )
                    torque_limit = float(self.get_parameter("torque_limit").value)
                    torque = float(np.clip(torque, -torque_limit, torque_limit))
                    for actuator_id in self.actuator_ids.get(name, []):
                        self.data.ctrl[actuator_id] = torque
                self.data.qvel[self.base_qvel["x"]] = self.base_command[0]
                self.data.qvel[self.base_qvel["y"]] = self.base_command[1]
                self.data.qvel[self.base_qvel["yaw"]] = self.base_command[2]
                mujoco.mj_step(self.model, self.data)
                self._update_physical_grasp_state()
            step_count += 1
            if step_count % state_interval == 0:
                self.publish_state()
            next_tick += period
            time.sleep(max(0.0, next_tick - time.monotonic()))

    def camera_loop(self):
        # Rendering has its own EGL context and a private MjData snapshot, so
        # expensive RGB-D rendering cannot stop the 500 Hz simulation/state
        # paths. The short copy under the lock keeps the live simulation data
        # protected while rendering happens outside the lock.
        renderer = mujoco.Renderer(
            self.model,
            height=int(self.get_parameter("camera_height").value),
            width=int(self.get_parameter("camera_width").value),
        )
        camera_data = mujoco.MjData(self.model)
        period = 1.0 / float(self.get_parameter("camera_hz").value)
        next_tick = time.monotonic()
        try:
            while self.running and rclpy.ok():
                with self.lock:
                    mujoco.mj_copyData(camera_data, self.model, self.data)
                try:
                    self.publish_camera(renderer, camera_data)
                except Exception:
                    # ROS may invalidate publishers before this worker sees
                    # the shutdown flag. Exit quietly during normal teardown.
                    if not rclpy.ok():
                        break
                    raise
                next_tick += period
                time.sleep(max(0.0, next_tick - time.monotonic()))
        finally:
            renderer.close()

    def publish_state(self):
        if not rclpy.ok():
            return
        with self.lock:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(self.control_names)
            msg.position = [float(self.data.qpos[self.qpos_addr[n]]) for n in self.control_names]
            msg.velocity = [float(self.data.qvel[self.qvel_addr[n]]) for n in self.control_names]
            box_pose = PoseStamped()
            box_pose.header.stamp = msg.header.stamp
            box_pose.header.frame_id = "base_link"
            box_pose.pose.position.x = float(self.data.xpos[self.box_body_id][0])
            box_pose.pose.position.y = float(self.data.xpos[self.box_body_id][1])
            box_pose.pose.position.z = float(self.data.xpos[self.box_body_id][2])
            box_quaternion = self.data.xquat[self.box_body_id]
            box_pose.pose.orientation.w = float(box_quaternion[0])
            box_pose.pose.orientation.x = float(box_quaternion[1])
            box_pose.pose.orientation.y = float(box_quaternion[2])
            box_pose.pose.orientation.z = float(box_quaternion[3])
            grasped = self.box_grasped
        if rclpy.ok():
            self.state_pub.publish(msg)
            self.box_state_pub.publish(box_pose)
            grasped_msg = Bool()
            grasped_msg.data = grasped
            self.box_grasped_pub.publish(grasped_msg)

    def publish_camera(self, renderer, data):
        renderer.update_scene(data, camera="gensong_rgb_camera")
        rgb = renderer.render()
        renderer.enable_depth_rendering()
        renderer.update_scene(data, camera="gensong_depth_camera")
        depth = renderer.render().astype(np.float32)
        renderer.disable_depth_rendering()
        stamp = self.get_clock().now().to_msg()
        self.color_pub.publish(self.image_message(rgb, "rgb8", stamp))
        self.depth_pub.publish(self.image_message(depth, "32FC1", stamp))
        for pub in (self.color_info_pub, self.depth_info_pub):
            info = CameraInfo(); info.header.stamp = stamp; info.width = rgb.shape[1]; info.height = rgb.shape[0]
            f = info.width / (2.0 * np.tan(np.deg2rad(70.0) / 2.0)); info.k = [f, 0, info.width / 2.0, 0, f, info.height / 2.0, 0, 0, 1]
            pub.publish(info)

    def image_message(self, array, encoding, stamp):
        msg = Image(); msg.header.stamp = stamp; msg.height, msg.width = array.shape[:2]
        msg.encoding = encoding; msg.is_bigendian = 0; msg.step = array.strides[0]; msg.data = np.ascontiguousarray(array).tobytes()
        return msg

    def destroy_node(self):
        self.running = False
        if hasattr(self, "sim_thread"): self.sim_thread.join(timeout=2.0)
        if hasattr(self, "camera_thread"): self.camera_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GensongMujoco()
    viewer = None
    try:
        if node.get_parameter("viewer").value:
            viewer = mujoco.viewer.launch_passive(node.model, node.data)
            while viewer.is_running() and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.01)
                with node.lock:
                    viewer.sync()
        else:
            rclpy.spin(node)
    finally:
        if viewer is not None:
            viewer.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
