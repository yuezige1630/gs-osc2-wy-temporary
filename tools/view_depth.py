import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np

class DepthViewer(Node):
    def __init__(self):
        super().__init__("depth_viewer")
        self.sub = self.create_subscription(
            Image, "/gensong/depth", self.cb, 10
        )

    def cb(self, msg: Image):
        # 根据 ROS Image encoding 还原正确numpy数组
        if msg.encoding == "16UC1":
            arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
        elif msg.encoding in ["rgb16", "bgr16"]:
            arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width, 3)
        elif msg.encoding == "32FC1":
            arr = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        else:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)

        self.get_logger().info(f"img shape: H={msg.height}, W={msg.width}, encoding={msg.encoding}, data_len={len(msg.data)}")

        # 处理深度可视化
        if len(arr.shape) == 2:
            # 单通道深度图归一化上色
            vis = cv2.normalize(arr, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
            vis = cv2.applyColorMap(vis, cv2.COLORMAP_JET)
        else:
            vis = arr

        cv2.imshow("depth_view", vis)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = DepthViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()