#!/usr/bin/python3

from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_srvs.srv import Trigger


@dataclass
class PoseConfig:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


@dataclass
class RigidTransform:
    translation: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]


def quaternion_normalize(quaternion: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        raise ValueError("Quaternion norm is too small.")
    return (x / norm, y / norm, z / norm, w / norm)


def quaternion_multiply(
    lhs: tuple[float, float, float, float],
    rhs: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = lhs
    rx, ry, rz, rw = rhs
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def quaternion_conjugate(quaternion: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    x, y, z, w = quaternion
    return (-x, -y, -z, w)


def rotate_vector(quaternion: tuple[float, float, float, float], vector: tuple[float, float, float]) -> tuple[float, float, float]:
    vector_quaternion = (vector[0], vector[1], vector[2], 0.0)
    rotated = quaternion_multiply(quaternion_multiply(quaternion, vector_quaternion), quaternion_conjugate(quaternion))
    return (rotated[0], rotated[1], rotated[2])


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr = math.cos(0.5 * roll)
    sr = math.sin(0.5 * roll)
    cp = math.cos(0.5 * pitch)
    sp = math.sin(0.5 * pitch)
    cy = math.cos(0.5 * yaw)
    sy = math.sin(0.5 * yaw)
    return quaternion_normalize(
        (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )
    )


def quaternion_to_rotation_matrix(quaternion: tuple[float, float, float, float]) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    x, y, z, w = quaternion_normalize(quaternion)
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


def normalize_vector(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    norm = math.sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2])
    if norm < 1e-9:
        raise ValueError("Vector norm is too small.")
    return (vector[0] / norm, vector[1] / norm, vector[2] / norm)


def cross(lhs: tuple[float, float, float], rhs: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        lhs[1] * rhs[2] - lhs[2] * rhs[1],
        lhs[2] * rhs[0] - lhs[0] * rhs[2],
        lhs[0] * rhs[1] - lhs[1] * rhs[0],
    )


def rotation_matrix_to_quaternion(
    rotation: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]
) -> tuple[float, float, float, float]:
    m00, m01, m02 = rotation[0]
    m10, m11, m12 = rotation[1]
    m20, m21, m22 = rotation[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m21 - m12) / s
        qy = (m02 - m20) / s
        qz = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / s
        qx = 0.25 * s
        qy = (m01 + m10) / s
        qz = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / s
        qx = (m01 + m10) / s
        qy = 0.25 * s
        qz = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / s
        qx = (m02 + m20) / s
        qy = (m12 + m21) / s
        qz = 0.25 * s
    return quaternion_normalize((qx, qy, qz, qw))


def quaternion_from_axes(
    x_axis: tuple[float, float, float],
    y_axis: tuple[float, float, float],
    z_axis: tuple[float, float, float],
) -> tuple[float, float, float, float]:
    x_axis = normalize_vector(x_axis)
    y_axis = normalize_vector(y_axis)
    z_axis = normalize_vector(z_axis)
    return rotation_matrix_to_quaternion(
        (
            (x_axis[0], y_axis[0], z_axis[0]),
            (x_axis[1], y_axis[1], z_axis[1]),
            (x_axis[2], y_axis[2], z_axis[2]),
        )
    )


def parse_xyz(value: str | None) -> tuple[float, float, float]:
    if not value:
        return (0.0, 0.0, 0.0)
    parts = [float(x) for x in value.split()]
    return (parts[0], parts[1], parts[2])


def load_wrist_to_hand_transform(
    urdf_path: str,
    wrist_link_name: str,
    preferred_child_link_name: str,
) -> RigidTransform:
    root = ET.parse(urdf_path).getroot()
    fallback = None
    for joint in root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            continue
        if parent.attrib.get("link") != wrist_link_name:
            continue
        child_link_name = child.attrib.get("link", "")
        if not child_link_name.startswith("hand_base"):
            continue
        origin = joint.find("origin")
        xyz = parse_xyz(None if origin is None else origin.attrib.get("xyz"))
        rpy = parse_xyz(None if origin is None else origin.attrib.get("rpy"))
        transform = RigidTransform(
            translation=xyz,
            quaternion_xyzw=rpy_to_quaternion(rpy[0], rpy[1], rpy[2]),
        )
        if child_link_name == preferred_child_link_name:
            return transform
        fallback = transform
    if fallback is not None:
        return fallback
    raise RuntimeError(f"Could not find fixed hand_base joint under {wrist_link_name} in {urdf_path}.")


class PresetDualArmGraspPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__("preset_dual_arm_grasp_pose_publisher")

        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("box_pose_topic", "box_pose")
        self.declare_parameter("left_grasp_pose_topic", "left_grasp_pose")
        self.declare_parameter("right_grasp_pose_topic", "right_grasp_pose")
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("auto_trigger", True)
        self.declare_parameter("trigger_delay_sec", 3.0)
        self.declare_parameter("planner_service_name", "plan_and_send_grasp_trajectory")
        self.declare_parameter("urdf_file", "")

        self.declare_parameter("box_center.x", 0.8994)
        self.declare_parameter("box_center.y", -0.0327)
        self.declare_parameter("box_center.z", 1.1000)
        self.declare_parameter("box_center.qx", 0.0)
        self.declare_parameter("box_center.qy", 0.0)
        self.declare_parameter("box_center.qz", 0.0)
        self.declare_parameter("box_center.qw", 1.0)

        self.declare_parameter("box_size_x", 0.1978)
        self.declare_parameter("box_size_y", 0.3029)
        self.declare_parameter("box_size_z", 0.1464)
        self.declare_parameter("grasp_edge_inset_y", 0.0)
        self.declare_parameter("grasp_x_offset", 0.0)
        self.declare_parameter("grasp_z_offset", 0.0)
        self.declare_parameter("grasp_frame_rpy.roll", 0.0)
        self.declare_parameter("grasp_frame_rpy.pitch", 0.0)
        self.declare_parameter("grasp_frame_rpy.yaw", 0.0)
        self.declare_parameter("left_wrist_compensation_rpy.roll", 0.0)
        self.declare_parameter("left_wrist_compensation_rpy.pitch", 0.0)
        self.declare_parameter("left_wrist_compensation_rpy.yaw", 0.0)
        self.declare_parameter("right_wrist_compensation_rpy.roll", 0.0)
        self.declare_parameter("right_wrist_compensation_rpy.pitch", 0.0)
        self.declare_parameter("right_wrist_compensation_rpy.yaw", 0.0)

        self.frame_id = self.get_parameter("frame_id").value
        self.box_pose_topic = self.get_parameter("box_pose_topic").value
        self.left_grasp_pose_topic = self.get_parameter("left_grasp_pose_topic").value
        self.right_grasp_pose_topic = self.get_parameter("right_grasp_pose_topic").value
        self.urdf_file = self.get_parameter("urdf_file").value
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.auto_trigger = bool(self.get_parameter("auto_trigger").value)
        trigger_delay_sec = float(self.get_parameter("trigger_delay_sec").value)
        self.planner_service_name = self.get_parameter("planner_service_name").value
        if not self.urdf_file:
            raise ValueError("Parameter 'urdf_file' is required to derive wrist-to-hand compensation from URDF.")

        self.box_pose = PoseConfig(
            x=float(self.get_parameter("box_center.x").value),
            y=float(self.get_parameter("box_center.y").value),
            z=float(self.get_parameter("box_center.z").value),
            qx=float(self.get_parameter("box_center.qx").value),
            qy=float(self.get_parameter("box_center.qy").value),
            qz=float(self.get_parameter("box_center.qz").value),
            qw=float(self.get_parameter("box_center.qw").value),
        )

        box_size_x = float(self.get_parameter("box_size_x").value)
        box_size_y = float(self.get_parameter("box_size_y").value)
        box_size_z = float(self.get_parameter("box_size_z").value)
        grasp_edge_inset_y = float(self.get_parameter("grasp_edge_inset_y").value)
        grasp_edge_offset_y = max(0.0, 0.5 * box_size_y - grasp_edge_inset_y)
        grasp_x_offset = float(self.get_parameter("grasp_x_offset").value)
        grasp_z_offset = float(self.get_parameter("grasp_z_offset").value)
        box_quaternion = quaternion_normalize(
            (self.box_pose.qx, self.box_pose.qy, self.box_pose.qz, self.box_pose.qw)
        )
        grasp_frame_quaternion = rpy_to_quaternion(
            float(self.get_parameter("grasp_frame_rpy.roll").value),
            float(self.get_parameter("grasp_frame_rpy.pitch").value),
            float(self.get_parameter("grasp_frame_rpy.yaw").value),
        )
        left_wrist_compensation = rpy_to_quaternion(
            float(self.get_parameter("left_wrist_compensation_rpy.roll").value),
            float(self.get_parameter("left_wrist_compensation_rpy.pitch").value),
            float(self.get_parameter("left_wrist_compensation_rpy.yaw").value),
        )
        right_wrist_compensation = rpy_to_quaternion(
            float(self.get_parameter("right_wrist_compensation_rpy.roll").value),
            float(self.get_parameter("right_wrist_compensation_rpy.pitch").value),
            float(self.get_parameter("right_wrist_compensation_rpy.yaw").value),
        )

        left_wrist_to_hand = load_wrist_to_hand_transform(self.urdf_file, "wrist3_left_link", "hand_base_link")
        right_wrist_to_hand = load_wrist_to_hand_transform(self.urdf_file, "wrist3_right_link", "hand_base_right_link")

        left_local_offset = (grasp_x_offset, grasp_edge_offset_y, grasp_z_offset)
        right_local_offset = (grasp_x_offset, -grasp_edge_offset_y, grasp_z_offset)
        left_offset_world = rotate_vector(box_quaternion, left_local_offset)
        right_offset_world = rotate_vector(box_quaternion, right_local_offset)
        left_grasp_quaternion = quaternion_normalize(
            quaternion_multiply(
                quaternion_multiply(box_quaternion, grasp_frame_quaternion),
                left_wrist_compensation,
            )
        )
        right_grasp_quaternion = quaternion_normalize(
            quaternion_multiply(
                quaternion_multiply(box_quaternion, grasp_frame_quaternion),
                right_wrist_compensation,
            )
        )

        left_wrist_offset_world = rotate_vector(left_grasp_quaternion, left_wrist_to_hand.translation)
        right_wrist_offset_world = rotate_vector(right_grasp_quaternion, right_wrist_to_hand.translation)
        left_hand_position = (
            self.box_pose.x + left_offset_world[0],
            self.box_pose.y + left_offset_world[1],
            self.box_pose.z + left_offset_world[2],
        )
        right_hand_position = (
            self.box_pose.x + right_offset_world[0],
            self.box_pose.y + right_offset_world[1],
            self.box_pose.z + right_offset_world[2],
        )

        self.left_grasp_pose = PoseConfig(
            x=left_hand_position[0] - left_wrist_offset_world[0],
            y=left_hand_position[1] - left_wrist_offset_world[1],
            z=left_hand_position[2] - left_wrist_offset_world[2],
            qx=left_grasp_quaternion[0],
            qy=left_grasp_quaternion[1],
            qz=left_grasp_quaternion[2],
            qw=left_grasp_quaternion[3],
        )
        self.right_grasp_pose = PoseConfig(
            x=right_hand_position[0] - right_wrist_offset_world[0],
            y=right_hand_position[1] - right_wrist_offset_world[1],
            z=right_hand_position[2] - right_wrist_offset_world[2],
            qx=right_grasp_quaternion[0],
            qy=right_grasp_quaternion[1],
            qz=right_grasp_quaternion[2],
            qw=right_grasp_quaternion[3],
        )

        self.box_pub = self.create_publisher(PoseStamped, self.box_pose_topic, 10)
        self.left_pub = self.create_publisher(PoseStamped, self.left_grasp_pose_topic, 10)
        self.right_pub = self.create_publisher(PoseStamped, self.right_grasp_pose_topic, 10)
        self.publish_timer = self.create_timer(1.0 / max(publish_rate_hz, 1e-3), self._publish_all)

        self.trigger_client = self.create_client(Trigger, self.planner_service_name)
        self.trigger_sent = False
        if self.auto_trigger:
            self.trigger_timer = self.create_timer(trigger_delay_sec, self._maybe_trigger_planner)
        else:
            self.trigger_timer = None

        self.get_logger().info(
            "Publishing preset box/grasp poses in frame '%s': "
            "box=(%.4f, %.4f, %.4f), size=(%.4f, %.4f, %.4f), "
            "left=(%.4f, %.4f, %.4f), right=(%.4f, %.4f, %.4f), edge_offset_y=%.4f, "
            "left_q=(%.4f, %.4f, %.4f, %.4f), right_q=(%.4f, %.4f, %.4f, %.4f)"
            % (
                self.frame_id,
                self.box_pose.x,
                self.box_pose.y,
                self.box_pose.z,
                box_size_x,
                box_size_y,
                box_size_z,
                self.left_grasp_pose.x,
                self.left_grasp_pose.y,
                self.left_grasp_pose.z,
                self.right_grasp_pose.x,
                self.right_grasp_pose.y,
                self.right_grasp_pose.z,
                grasp_edge_offset_y,
                self.left_grasp_pose.qx,
                self.left_grasp_pose.qy,
                self.left_grasp_pose.qz,
                self.left_grasp_pose.qw,
                self.right_grasp_pose.qx,
                self.right_grasp_pose.qy,
                self.right_grasp_pose.qz,
                self.right_grasp_pose.qw,
            )
        )

    def _pose_to_msg(self, pose: PoseConfig) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = pose.x
        msg.pose.position.y = pose.y
        msg.pose.position.z = pose.z
        msg.pose.orientation.x = pose.qx
        msg.pose.orientation.y = pose.qy
        msg.pose.orientation.z = pose.qz
        msg.pose.orientation.w = pose.qw
        return msg

    def _publish_all(self) -> None:
        self.box_pub.publish(self._pose_to_msg(self.box_pose))
        self.left_pub.publish(self._pose_to_msg(self.left_grasp_pose))
        self.right_pub.publish(self._pose_to_msg(self.right_grasp_pose))

    def _maybe_trigger_planner(self) -> None:
        if self.trigger_sent:
            return
        if not self.trigger_client.wait_for_service(timeout_sec=0.0):
            self.get_logger().info("Waiting for planner service '%s'..." % self.planner_service_name)
            return

        request = Trigger.Request()
        future = self.trigger_client.call_async(request)
        future.add_done_callback(self._handle_trigger_response)
        self.trigger_sent = True
        if self.trigger_timer is not None:
            self.destroy_timer(self.trigger_timer)
            self.trigger_timer = None
        self.get_logger().info("Sent planning trigger to '%s'." % self.planner_service_name)

    def _handle_trigger_response(self, future) -> None:
        try:
            response = future.result()
            if response is None:
                self.get_logger().error("Planner service returned no response.")
                return
            level = self.get_logger().info if response.success else self.get_logger().error
            level("Planner response: %s" % response.message)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error("Planner service call failed: %s" % exc)


def main() -> None:
    rclpy.init()
    node = PresetDualArmGraspPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
