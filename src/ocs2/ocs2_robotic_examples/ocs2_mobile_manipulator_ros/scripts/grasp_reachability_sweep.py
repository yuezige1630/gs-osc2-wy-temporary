#!/usr/bin/python3

from __future__ import annotations

from dataclasses import dataclass
import csv
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from ocs2_msgs.srv import EvaluateBoxPose


@dataclass(frozen=True)
class SweepSample:
    x: float
    y: float
    z: float
    success: bool
    message: str
    qx: float
    qy: float
    qz: float
    qw: float


def quaternion_normalize(quaternion: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError("Quaternion norm is too small.")
    return (x / norm, y / norm, z / norm, w / norm)


def inclusive_range(start: float, stop: float, step: float) -> list[float]:
    if not math.isfinite(start) or not math.isfinite(stop) or not math.isfinite(step):
        raise ValueError("Sweep bounds must be finite.")
    if step <= 0.0:
        raise ValueError("Sweep step must be positive.")
    if stop < start:
        raise ValueError("Sweep upper bound must be greater than or equal to lower bound.")

    values: list[float] = []
    current = start
    while current <= stop + 1e-9:
        values.append(round(current, 10))
        current += step
    return values


class GraspReachabilitySweepNode(Node):
    def __init__(self) -> None:
        super().__init__("grasp_reachability_sweep")

        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("box_pose_topic", "box_pose")
        self.declare_parameter("evaluate_service_name", "evaluate_box_pose")
        self.declare_parameter("csv_path", "/tmp/gensong_grasp_reachability.csv")
        self.declare_parameter("summary_path", "")
        self.declare_parameter("publish_box_pose", True)
        self.declare_parameter("pose_publish_settle_sec", 0.05)
        self.declare_parameter("service_timeout_sec", 10.0)
        self.declare_parameter("service_wait_timeout_sec", 30.0)
        self.declare_parameter("startup_wait_sec", 2.0)

        self.declare_parameter("box_qx", 0.0)
        self.declare_parameter("box_qy", 0.0)
        self.declare_parameter("box_qz", 0.0)
        self.declare_parameter("box_qw", 1.0)

        self.declare_parameter("x_min", 0.80)
        self.declare_parameter("x_max", 1.00)
        self.declare_parameter("x_step", 0.05)
        self.declare_parameter("y_min", -0.15)
        self.declare_parameter("y_max", 0.15)
        self.declare_parameter("y_step", 0.05)
        self.declare_parameter("z_min", 1.00)
        self.declare_parameter("z_max", 1.20)
        self.declare_parameter("z_step", 0.05)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.box_pose_topic = str(self.get_parameter("box_pose_topic").value)
        self.evaluate_service_name = str(self.get_parameter("evaluate_service_name").value)
        self.csv_path = Path(str(self.get_parameter("csv_path").value)).expanduser()
        summary_path_value = str(self.get_parameter("summary_path").value)
        self.summary_path = (
            Path(summary_path_value).expanduser()
            if summary_path_value
            else self.csv_path.with_name(f"{self.csv_path.stem}.summary.txt")
        )
        self.publish_box_pose = bool(self.get_parameter("publish_box_pose").value)
        self.pose_publish_settle_sec = float(self.get_parameter("pose_publish_settle_sec").value)
        self.service_timeout_sec = float(self.get_parameter("service_timeout_sec").value)
        self.service_wait_timeout_sec = float(self.get_parameter("service_wait_timeout_sec").value)
        self.startup_wait_sec = float(self.get_parameter("startup_wait_sec").value)

        self.box_orientation = quaternion_normalize(
            (
                float(self.get_parameter("box_qx").value),
                float(self.get_parameter("box_qy").value),
                float(self.get_parameter("box_qz").value),
                float(self.get_parameter("box_qw").value),
            )
        )

        self.x_values = inclusive_range(
            float(self.get_parameter("x_min").value),
            float(self.get_parameter("x_max").value),
            float(self.get_parameter("x_step").value),
        )
        self.x_step = float(self.get_parameter("x_step").value)
        self.y_values = inclusive_range(
            float(self.get_parameter("y_min").value),
            float(self.get_parameter("y_max").value),
            float(self.get_parameter("y_step").value),
        )
        self.y_step = float(self.get_parameter("y_step").value)
        self.z_values = inclusive_range(
            float(self.get_parameter("z_min").value),
            float(self.get_parameter("z_max").value),
            float(self.get_parameter("z_step").value),
        )
        self.z_step = float(self.get_parameter("z_step").value)

        self.pose_pub = self.create_publisher(PoseStamped, self.box_pose_topic, 10) if self.publish_box_pose else None
        self.evaluate_client = self.create_client(EvaluateBoxPose, self.evaluate_service_name)
        self.samples: list[SweepSample] = []

        self.get_logger().info(
            "Sweep configured: frame_id=%s, service=%s, csv=%s, grid=(%d x %d x %d)"
            % (
                self.frame_id,
                self.evaluate_service_name,
                str(self.csv_path),
                len(self.x_values),
                len(self.y_values),
                len(self.z_values),
            )
        )

    def build_pose(self, x: float, y: float, z: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = self.box_orientation[0]
        pose.pose.orientation.y = self.box_orientation[1]
        pose.pose.orientation.z = self.box_orientation[2]
        pose.pose.orientation.w = self.box_orientation[3]
        return pose

    def sleep_with_spin(self, duration_sec: float) -> None:
        deadline = time.monotonic() + max(0.0, duration_sec)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.01)

    def wait_for_service(self) -> None:
        deadline = time.monotonic() + max(0.0, self.service_wait_timeout_sec)
        while rclpy.ok() and time.monotonic() < deadline:
            if self.evaluate_client.wait_for_service(timeout_sec=0.2):
                return
            rclpy.spin_once(self, timeout_sec=0.0)
        raise RuntimeError(f"Service {self.evaluate_service_name} was not available in time.")

    def call_evaluation(self, pose: PoseStamped) -> tuple[bool, str]:
        request = EvaluateBoxPose.Request()
        request.box_pose = pose

        future = self.evaluate_client.call_async(request)
        deadline = time.monotonic() + max(0.0, self.service_timeout_sec)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.0)
            if future.done():
                response = future.result()
                if response is None:
                    return False, "Evaluation service returned no response."
                return bool(response.success), str(response.message)
            time.sleep(0.01)

        return False, f"Timed out waiting for {self.evaluate_service_name}."

    def run_sweep(self) -> None:
        self.wait_for_service()
        self.sleep_with_spin(self.startup_wait_sec)

        total = len(self.x_values) * len(self.y_values) * len(self.z_values)
        completed = 0
        for x in self.x_values:
            for y in self.y_values:
                for z in self.z_values:
                    completed += 1
                    pose = self.build_pose(x, y, z)
                    if self.pose_pub is not None:
                        self.pose_pub.publish(pose)
                        self.sleep_with_spin(self.pose_publish_settle_sec)

                    success, message = self.call_evaluation(pose)
                    self.samples.append(
                        SweepSample(
                            x=x,
                            y=y,
                            z=z,
                            success=success,
                            message=message,
                            qx=self.box_orientation[0],
                            qy=self.box_orientation[1],
                            qz=self.box_orientation[2],
                            qw=self.box_orientation[3],
                        )
                    )

                    status = "OK" if success else "FAIL"
                    self.get_logger().info(
                        "[%d/%d] x=%.4f y=%.4f z=%.4f -> %s"
                        % (completed, total, x, y, z, status)
                    )
                    if not success:
                        self.get_logger().warn(message)

        self.write_csv()
        self.write_summary()

    def write_csv(self) -> None:
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        with self.csv_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(
                csv_file,
                fieldnames=["x", "y", "z", "qx", "qy", "qz", "qw", "success", "message"],
            )
            writer.writeheader()
            for sample in self.samples:
                writer.writerow(
                    {
                        "x": sample.x,
                        "y": sample.y,
                        "z": sample.z,
                        "qx": sample.qx,
                        "qy": sample.qy,
                        "qz": sample.qz,
                        "qw": sample.qw,
                        "success": int(sample.success),
                        "message": sample.message,
                    }
                )
        self.get_logger().info("Wrote CSV results to %s" % self.csv_path)

    def axis_bounds(self, axis: str) -> tuple[float | None, float | None]:
        successful_values = [getattr(sample, axis) for sample in self.samples if sample.success]
        if not successful_values:
            return None, None
        return min(successful_values), max(successful_values)

    def safe_axis_bounds(self, axis: str, step: float) -> tuple[float | None, float | None]:
        lower, upper = self.axis_bounds(axis)
        if lower is None or upper is None:
            return None, None
        if upper - lower < 2.0 * step - 1e-9:
            return None, None
        return lower + step, upper - step

    def nearest_failed_bound(self, axis: str, lower: bool) -> float | None:
        successful_values = [getattr(sample, axis) for sample in self.samples if sample.success]
        failed_values = [getattr(sample, axis) for sample in self.samples if not sample.success]
        if not successful_values:
            return None

        success_min = min(successful_values)
        success_max = max(successful_values)
        if lower:
            candidates = [value for value in failed_values if value < success_min - 1e-9]
            return max(candidates) if candidates else None
        candidates = [value for value in failed_values if value > success_max + 1e-9]
        return min(candidates) if candidates else None

    def write_summary(self) -> None:
        success_count = sum(1 for sample in self.samples if sample.success)
        failure_count = len(self.samples) - success_count
        x_min, x_max = self.axis_bounds("x")
        y_min, y_max = self.axis_bounds("y")
        z_min, z_max = self.axis_bounds("z")
        safe_x_min, safe_x_max = self.safe_axis_bounds("x", self.x_step)
        safe_y_min, safe_y_max = self.safe_axis_bounds("y", self.y_step)
        safe_z_min, safe_z_max = self.safe_axis_bounds("z", self.z_step)

        lines = [
            "Gensong grasp reachability sweep summary",
            f"Total samples: {len(self.samples)}",
            f"Success: {success_count}",
            f"Failure: {failure_count}",
            "",
            "Success envelope:",
            f"  x: {self.format_interval(x_min, x_max)}",
            f"  y: {self.format_interval(y_min, y_max)}",
            f"  z: {self.format_interval(z_min, z_max)}",
            "",
            "Safe range:",
            f"  x: {self.format_interval(safe_x_min, safe_x_max)}",
            f"  y: {self.format_interval(safe_y_min, safe_y_max)}",
            f"  z: {self.format_interval(safe_z_min, safe_z_max)}",
            "",
            "Nearest failed boundary outside success envelope:",
            f"  x lower: {self.format_value(self.nearest_failed_bound('x', True))}",
            f"  x upper: {self.format_value(self.nearest_failed_bound('x', False))}",
            f"  y lower: {self.format_value(self.nearest_failed_bound('y', True))}",
            f"  y upper: {self.format_value(self.nearest_failed_bound('y', False))}",
            f"  z lower: {self.format_value(self.nearest_failed_bound('z', True))}",
            f"  z upper: {self.format_value(self.nearest_failed_bound('z', False))}",
        ]

        self.summary_path.parent.mkdir(parents=True, exist_ok=True)
        self.summary_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        self.get_logger().info("Wrote summary to %s" % self.summary_path)

    @staticmethod
    def format_value(value: float | None) -> str:
        return "none" if value is None else f"{value:.4f}"

    @staticmethod
    def format_interval(lower: float | None, upper: float | None) -> str:
        if lower is None or upper is None:
            return "none"
        return f"[{lower:.4f}, {upper:.4f}]"


def main() -> None:
    rclpy.init()
    node = GraspReachabilitySweepNode()
    try:
        node.run_sweep()
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(str(exc))
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
