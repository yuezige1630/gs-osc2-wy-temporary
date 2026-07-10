#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from PIL import Image as PilImage


class Capture(Node):
    def __init__(self):
        super().__init__("capture_mujoco_image")
        self.started = time.monotonic()
        self.saved = False
        self.sub = self.create_subscription(
            Image, "/gensong/camera/color/image_raw", self.callback, 10
        )

    def callback(self, msg):
        if self.saved or time.monotonic() - self.started < 12.0:
            return
        if msg.encoding != "rgb8":
            return
        image = PilImage.frombytes("RGB", (msg.width, msg.height), bytes(msg.data))
        image.save("/tmp/mujoco_box_lift.png")
        self.saved = True
        self.get_logger().info("saved /tmp/mujoco_box_lift.png")


rclpy.init()
node = Capture()
while rclpy.ok() and not node.saved and time.monotonic() - node.started < 25.0:
    rclpy.spin_once(node, timeout_sec=0.1)
node.destroy_node()
rclpy.shutdown()
