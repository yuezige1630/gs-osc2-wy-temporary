#!/usr/bin/python3

from __future__ import annotations

from dataclasses import dataclass
import csv
import math
import random
import time
from pathlib import Path

import rclpy
from ocs2_msgs.msg import MpcInput, MpcObservation, MpcState, MpcTargetTrajectories
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass(frozen=True)
class PoseConfig:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


@dataclass(frozen=True)
class PoseError:
    dx_mm: float
    dy_mm: float
    dz_mm: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    angle_deg: float


def quaternion_normalize(quaternion: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
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


def quaternion_to_rpy(quaternion: tuple[float, float, float, float]) -> tuple[float, float, float]:
    x, y, z, w = quaternion_normalize(quaternion)

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


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


def pose_to_state_segment(pose: PoseConfig) -> list[float]:
    return [pose.x, pose.y, pose.z, pose.qx, pose.qy, pose.qz, pose.qw]


def pose_error(target: PoseConfig, actual: PoseConfig) -> PoseError:
    dx_mm = (actual.x - target.x) * 1000.0
    dy_mm = (actual.y - target.y) * 1000.0
    dz_mm = (actual.z - target.z) * 1000.0

    q_target = quaternion_normalize((target.qx, target.qy, target.qz, target.qw))
    q_actual = quaternion_normalize((actual.qx, actual.qy, actual.qz, actual.qw))
    q_error = quaternion_multiply(quaternion_conjugate(q_target), q_actual)
    q_error = quaternion_normalize(q_error)
    if q_error[3] < 0.0:
        q_error = tuple(-value for value in q_error)

    roll, pitch, yaw = quaternion_to_rpy(q_error)
    angle = 2.0 * math.atan2(math.sqrt(q_error[0] * q_error[0] + q_error[1] * q_error[1] + q_error[2] * q_error[2]), q_error[3])
    return PoseError(
        dx_mm=dx_mm,
        dy_mm=dy_mm,
        dz_mm=dz_mm,
        roll_deg=math.degrees(roll),
        pitch_deg=math.degrees(pitch),
        yaw_deg=math.degrees(yaw),
        angle_deg=math.degrees(angle),
    )


class DualArmTrackingErrorTester(Node):
    def __init__(self) -> None:
        super().__init__("dual_arm_tracking_error_tester")

        self.declare_parameter("world_frame", "world")
        self.declare_parameter("left_frame", "handboard_left")
        self.declare_parameter("right_frame", "handboard_right")
        self.declare_parameter("observation_topic", "mobile_manipulator_mpc_observation")
        self.declare_parameter("target_topic", "mobile_manipulator_mpc_target")
        self.declare_parameter("sample_count", 5)
        self.declare_parameter("seed", 7)
        self.declare_parameter("pos_offset_x_mm", 20.0)
        self.declare_parameter("pos_offset_y_mm", 20.0)
        self.declare_parameter("pos_offset_z_mm", 20.0)
        self.declare_parameter("rot_offset_roll_deg", 5.0)
        self.declare_parameter("rot_offset_pitch_deg", 5.0)
        self.declare_parameter("rot_offset_yaw_deg", 5.0)
        self.declare_parameter("settle_delay_sec", 3.0)
        self.declare_parameter("measure_duration_sec", 1.0)
        self.declare_parameter("measure_rate_hz", 20.0)
        self.declare_parameter("csv_path", "")
        self.declare_parameter("same_offset_for_both_hands", False)

        self.world_frame = str(self.get_parameter("world_frame").value)
        self.left_frame = str(self.get_parameter("left_frame").value)
        self.right_frame = str(self.get_parameter("right_frame").value)
        self.observation_topic = str(self.get_parameter("observation_topic").value)
        self.target_topic = str(self.get_parameter("target_topic").value)
        self.sample_count = int(self.get_parameter("sample_count").value)
        self.seed = int(self.get_parameter("seed").value)
        self.pos_offset_ranges_m = (
            float(self.get_parameter("pos_offset_x_mm").value) / 1000.0,
            float(self.get_parameter("pos_offset_y_mm").value) / 1000.0,
            float(self.get_parameter("pos_offset_z_mm").value) / 1000.0,
        )
        self.rot_offset_ranges_rad = (
            math.radians(float(self.get_parameter("rot_offset_roll_deg").value)),
            math.radians(float(self.get_parameter("rot_offset_pitch_deg").value)),
            math.radians(float(self.get_parameter("rot_offset_yaw_deg").value)),
        )
        self.settle_delay_sec = float(self.get_parameter("settle_delay_sec").value)
        self.measure_duration_sec = float(self.get_parameter("measure_duration_sec").value)
        self.measure_rate_hz = max(1.0, float(self.get_parameter("measure_rate_hz").value))
        self.csv_path = str(self.get_parameter("csv_path").value)
        self.same_offset_for_both_hands = bool(self.get_parameter("same_offset_for_both_hands").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.observation_sub = self.create_subscription(
            MpcObservation, self.observation_topic, self._observation_callback, 10
        )
        self.target_pub = self.create_publisher(MpcTargetTrajectories, self.target_topic, 10)
        self.rng = random.Random(self.seed)

        self._csv_rows: list[dict[str, float | int | str]] = []
        self.latest_observation_time: float | None = None
        self.latest_input_size: int | None = None

    def _observation_callback(self, msg: MpcObservation) -> None:
        self.latest_observation_time = float(msg.time)
        self.latest_input_size = len(msg.input.value)

    def lookup_pose(self, child_frame: str) -> PoseConfig:
        transform = self.tf_buffer.lookup_transform(self.world_frame, child_frame, rclpy.time.Time())
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        qx, qy, qz, qw = quaternion_normalize((rotation.x, rotation.y, rotation.z, rotation.w))
        return PoseConfig(
            x=float(translation.x),
            y=float(translation.y),
            z=float(translation.z),
            qx=qx,
            qy=qy,
            qz=qz,
            qw=qw,
        )

    def wait_for_initial_poses(self) -> tuple[PoseConfig, PoseConfig]:
        self.get_logger().info(
            "Waiting for transforms %s->%s and %s->%s..."
            % (self.world_frame, self.left_frame, self.world_frame, self.right_frame)
        )
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                left_pose = self.lookup_pose(self.left_frame)
                right_pose = self.lookup_pose(self.right_frame)
            except TransformException:
                continue
            if self.latest_input_size is None:
                continue
            return left_pose, right_pose
        raise RuntimeError("ROS was shut down before initial poses became available.")

    def sleep_with_spin(self, duration_sec: float) -> None:
        deadline = time.monotonic() + max(0.0, duration_sec)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.02)

    def sample_delta(self) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        dx = self.rng.uniform(-self.pos_offset_ranges_m[0], self.pos_offset_ranges_m[0])
        dy = self.rng.uniform(-self.pos_offset_ranges_m[1], self.pos_offset_ranges_m[1])
        dz = self.rng.uniform(-self.pos_offset_ranges_m[2], self.pos_offset_ranges_m[2])
        roll = self.rng.uniform(-self.rot_offset_ranges_rad[0], self.rot_offset_ranges_rad[0])
        pitch = self.rng.uniform(-self.rot_offset_ranges_rad[1], self.rot_offset_ranges_rad[1])
        yaw = self.rng.uniform(-self.rot_offset_ranges_rad[2], self.rot_offset_ranges_rad[2])
        return (dx, dy, dz), (roll, pitch, yaw)

    def offset_pose(self, base_pose: PoseConfig, pos_delta: tuple[float, float, float], rpy_delta: tuple[float, float, float]) -> PoseConfig:
        base_quaternion = quaternion_normalize((base_pose.qx, base_pose.qy, base_pose.qz, base_pose.qw))
        delta_quaternion = rpy_to_quaternion(*rpy_delta)
        target_quaternion = quaternion_multiply(base_quaternion, delta_quaternion)
        target_quaternion = quaternion_normalize(target_quaternion)
        return PoseConfig(
            x=base_pose.x + pos_delta[0],
            y=base_pose.y + pos_delta[1],
            z=base_pose.z + pos_delta[2],
            qx=target_quaternion[0],
            qy=target_quaternion[1],
            qz=target_quaternion[2],
            qw=target_quaternion[3],
        )

    def publish_target(self, left_target: PoseConfig, right_target: PoseConfig) -> None:
        if self.latest_input_size is None:
            raise RuntimeError("No MPC observation received yet, input size is unknown.")

        message = MpcTargetTrajectories()
        message.time_trajectory = [self.latest_observation_time or (float(self.get_clock().now().nanoseconds) * 1.0e-9)]

        state = MpcState()
        state.value = pose_to_state_segment(left_target) + pose_to_state_segment(right_target)
        message.state_trajectory = [state]

        input_msg = MpcInput()
        input_msg.value = [0.0] * self.latest_input_size
        message.input_trajectory = [input_msg]
        self.target_pub.publish(message)

    def collect_error_window(self, left_target: PoseConfig, right_target: PoseConfig) -> tuple[list[PoseError], list[PoseError]]:
        left_errors: list[PoseError] = []
        right_errors: list[PoseError] = []
        deadline = time.monotonic() + self.measure_duration_sec
        loop_sleep = 1.0 / self.measure_rate_hz

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.0)
            try:
                left_actual = self.lookup_pose(self.left_frame)
                right_actual = self.lookup_pose(self.right_frame)
            except TransformException:
                time.sleep(loop_sleep)
                continue
            left_errors.append(pose_error(left_target, left_actual))
            right_errors.append(pose_error(right_target, right_actual))
            time.sleep(loop_sleep)

        if not left_errors or not right_errors:
            raise RuntimeError("No TF samples were collected during the measurement window.")
        return left_errors, right_errors

    @staticmethod
    def mean_error(errors: list[PoseError]) -> PoseError:
        count = float(len(errors))
        return PoseError(
            dx_mm=sum(error.dx_mm for error in errors) / count,
            dy_mm=sum(error.dy_mm for error in errors) / count,
            dz_mm=sum(error.dz_mm for error in errors) / count,
            roll_deg=sum(error.roll_deg for error in errors) / count,
            pitch_deg=sum(error.pitch_deg for error in errors) / count,
            yaw_deg=sum(error.yaw_deg for error in errors) / count,
            angle_deg=sum(error.angle_deg for error in errors) / count,
        )

    def maybe_write_csv(self) -> None:
        if not self.csv_path:
            return
        output_path = Path(self.csv_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        if not self._csv_rows:
            return
        fieldnames = list(self._csv_rows[0].keys())
        with output_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self._csv_rows)
        self.get_logger().info(f"Wrote CSV results to {output_path}")

    def run(self) -> None:
        initial_left, initial_right = self.wait_for_initial_poses()

        self.get_logger().info(
            "Initial left pose:  xyz=(%.4f, %.4f, %.4f)  q=(%.4f, %.4f, %.4f, %.4f)"
            % (initial_left.x, initial_left.y, initial_left.z, initial_left.qx, initial_left.qy, initial_left.qz, initial_left.qw)
        )
        self.get_logger().info(
            "Initial right pose: xyz=(%.4f, %.4f, %.4f)  q=(%.4f, %.4f, %.4f, %.4f)"
            % (initial_right.x, initial_right.y, initial_right.z, initial_right.qx, initial_right.qy, initial_right.qz, initial_right.qw)
        )
        self.get_logger().info(
            "Sampling %d target points with position ranges [%.1f, %.1f, %.1f] mm and rotation ranges [%.1f, %.1f, %.1f] deg."
            % (
                self.sample_count,
                self.pos_offset_ranges_m[0] * 1000.0,
                self.pos_offset_ranges_m[1] * 1000.0,
                self.pos_offset_ranges_m[2] * 1000.0,
                math.degrees(self.rot_offset_ranges_rad[0]),
                math.degrees(self.rot_offset_ranges_rad[1]),
                math.degrees(self.rot_offset_ranges_rad[2]),
            )
        )

        summary_rows: list[dict[str, float]] = []

        for sample_index in range(self.sample_count):
            left_delta_pos, left_delta_rpy = self.sample_delta()
            if self.same_offset_for_both_hands:
                right_delta_pos, right_delta_rpy = left_delta_pos, left_delta_rpy
            else:
                right_delta_pos, right_delta_rpy = self.sample_delta()

            left_target = self.offset_pose(initial_left, left_delta_pos, left_delta_rpy)
            right_target = self.offset_pose(initial_right, right_delta_pos, right_delta_rpy)

            self.publish_target(left_target, right_target)
            self.get_logger().info(
                "[%d/%d] Published target. Settling for %.2f s..."
                % (sample_index + 1, self.sample_count, self.settle_delay_sec)
            )
            self.sleep_with_spin(self.settle_delay_sec)

            left_errors, right_errors = self.collect_error_window(left_target, right_target)
            left_mean = self.mean_error(left_errors)
            right_mean = self.mean_error(right_errors)

            print(
                "[sample %d] left offset(mm)=(%.1f, %.1f, %.1f) ori_offset(deg)=(%.2f, %.2f, %.2f) | "
                "left mean err(mm)=(%.2f, %.2f, %.2f) ori_err(deg)=(%.2f, %.2f, %.2f) angle=%.2f"
                % (
                    sample_index + 1,
                    left_delta_pos[0] * 1000.0,
                    left_delta_pos[1] * 1000.0,
                    left_delta_pos[2] * 1000.0,
                    math.degrees(left_delta_rpy[0]),
                    math.degrees(left_delta_rpy[1]),
                    math.degrees(left_delta_rpy[2]),
                    left_mean.dx_mm,
                    left_mean.dy_mm,
                    left_mean.dz_mm,
                    left_mean.roll_deg,
                    left_mean.pitch_deg,
                    left_mean.yaw_deg,
                    left_mean.angle_deg,
                )
            )
            print(
                "[sample %d] right offset(mm)=(%.1f, %.1f, %.1f) ori_offset(deg)=(%.2f, %.2f, %.2f) | "
                "right mean err(mm)=(%.2f, %.2f, %.2f) ori_err(deg)=(%.2f, %.2f, %.2f) angle=%.2f"
                % (
                    sample_index + 1,
                    right_delta_pos[0] * 1000.0,
                    right_delta_pos[1] * 1000.0,
                    right_delta_pos[2] * 1000.0,
                    math.degrees(right_delta_rpy[0]),
                    math.degrees(right_delta_rpy[1]),
                    math.degrees(right_delta_rpy[2]),
                    right_mean.dx_mm,
                    right_mean.dy_mm,
                    right_mean.dz_mm,
                    right_mean.roll_deg,
                    right_mean.pitch_deg,
                    right_mean.yaw_deg,
                    right_mean.angle_deg,
                )
            )

            self._csv_rows.append(
                {
                    "sample": sample_index + 1,
                    "arm": "left",
                    "target_x_mm": left_target.x * 1000.0,
                    "target_y_mm": left_target.y * 1000.0,
                    "target_z_mm": left_target.z * 1000.0,
                    "target_qx": left_target.qx,
                    "target_qy": left_target.qy,
                    "target_qz": left_target.qz,
                    "target_qw": left_target.qw,
                    "offset_x_mm": left_delta_pos[0] * 1000.0,
                    "offset_y_mm": left_delta_pos[1] * 1000.0,
                    "offset_z_mm": left_delta_pos[2] * 1000.0,
                    "offset_roll_deg": math.degrees(left_delta_rpy[0]),
                    "offset_pitch_deg": math.degrees(left_delta_rpy[1]),
                    "offset_yaw_deg": math.degrees(left_delta_rpy[2]),
                    "mean_err_x_mm": left_mean.dx_mm,
                    "mean_err_y_mm": left_mean.dy_mm,
                    "mean_err_z_mm": left_mean.dz_mm,
                    "mean_err_roll_deg": left_mean.roll_deg,
                    "mean_err_pitch_deg": left_mean.pitch_deg,
                    "mean_err_yaw_deg": left_mean.yaw_deg,
                    "mean_err_angle_deg": left_mean.angle_deg,
                }
            )
            self._csv_rows.append(
                {
                    "sample": sample_index + 1,
                    "arm": "right",
                    "target_x_mm": right_target.x * 1000.0,
                    "target_y_mm": right_target.y * 1000.0,
                    "target_z_mm": right_target.z * 1000.0,
                    "target_qx": right_target.qx,
                    "target_qy": right_target.qy,
                    "target_qz": right_target.qz,
                    "target_qw": right_target.qw,
                    "offset_x_mm": right_delta_pos[0] * 1000.0,
                    "offset_y_mm": right_delta_pos[1] * 1000.0,
                    "offset_z_mm": right_delta_pos[2] * 1000.0,
                    "offset_roll_deg": math.degrees(right_delta_rpy[0]),
                    "offset_pitch_deg": math.degrees(right_delta_rpy[1]),
                    "offset_yaw_deg": math.degrees(right_delta_rpy[2]),
                    "mean_err_x_mm": right_mean.dx_mm,
                    "mean_err_y_mm": right_mean.dy_mm,
                    "mean_err_z_mm": right_mean.dz_mm,
                    "mean_err_roll_deg": right_mean.roll_deg,
                    "mean_err_pitch_deg": right_mean.pitch_deg,
                    "mean_err_yaw_deg": right_mean.yaw_deg,
                    "mean_err_angle_deg": right_mean.angle_deg,
                }
            )

            summary_rows.append(
                {
                    "left_pos_norm_mm": math.sqrt(
                        left_mean.dx_mm * left_mean.dx_mm
                        + left_mean.dy_mm * left_mean.dy_mm
                        + left_mean.dz_mm * left_mean.dz_mm
                    ),
                    "right_pos_norm_mm": math.sqrt(
                        right_mean.dx_mm * right_mean.dx_mm
                        + right_mean.dy_mm * right_mean.dy_mm
                        + right_mean.dz_mm * right_mean.dz_mm
                    ),
                    "left_angle_deg": left_mean.angle_deg,
                    "right_angle_deg": right_mean.angle_deg,
                }
            )

        if summary_rows:
            left_pos_avg = sum(row["left_pos_norm_mm"] for row in summary_rows) / len(summary_rows)
            right_pos_avg = sum(row["right_pos_norm_mm"] for row in summary_rows) / len(summary_rows)
            left_angle_avg = sum(row["left_angle_deg"] for row in summary_rows) / len(summary_rows)
            right_angle_avg = sum(row["right_angle_deg"] for row in summary_rows) / len(summary_rows)
            print(
                "[summary] left avg position error norm = %.2f mm, left avg angle error = %.2f deg"
                % (left_pos_avg, left_angle_avg)
            )
            print(
                "[summary] right avg position error norm = %.2f mm, right avg angle error = %.2f deg"
                % (right_pos_avg, right_angle_avg)
            )

        self.maybe_write_csv()


def main() -> None:
    rclpy.init()
    node = DualArmTrackingErrorTester()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
