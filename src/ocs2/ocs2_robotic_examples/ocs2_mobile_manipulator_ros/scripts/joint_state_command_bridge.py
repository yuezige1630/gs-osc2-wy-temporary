#!/usr/bin/python3

"""Forward /joint_states to /gensong/joint_command at a fixed 50 Hz."""

from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState


class JointStateCommandBridge(Node):
    def __init__(self) -> None:
        super().__init__("joint_state_command_bridge")
        self.declare_parameter("input_topic", "/joint_states")
        self.declare_parameter("output_topic", "/gensong/joint_command")

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self._publish_period_sec = 1.0 / 50.0
        self._latest_msg = None

        self._publisher = self.create_publisher(JointState, output_topic, QoSProfile(depth=10))
        self._subscription = self.create_subscription(
            JointState,
            input_topic,
            self._callback,
            qos_profile_sensor_data,
        )
        self._timer = self.create_timer(self._publish_period_sec, self._on_timer)

        self.get_logger().info(
            f"Forwarding {input_topic} -> {output_topic} at 50 Hz as sensor_msgs/msg/JointState"
        )

    def _callback(self, msg: JointState) -> None:
        self._latest_msg = deepcopy(msg)

    def _on_timer(self) -> None:
        if self._latest_msg is None:
            return
        self._publisher.publish(self._latest_msg)


def main() -> None:
    rclpy.init()
    node = JointStateCommandBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
