#!/usr/bin/env python3
"""
订阅 /gensong/depth_pcl (PointCloud2) 并用 matplotlib 3D 散点图可视化。

显示数据在原始相机坐标系中的位置，并支持通过 TF 转换到 world 坐标系。

用法:
  ROS_DOMAIN_ID=88 python3 tools/plot_depth_pcl_3d.py              # 相机原始坐标系
  ROS_DOMAIN_ID=88 python3 tools/plot_depth_pcl_3d.py --world      # 转换到 world 坐标系
"""

import sys
import argparse
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener


class DepthPCL3DViewer(Node):
    def __init__(self):
        super().__init__("depth_pcl_3d_viewer")
        self.sub = self.create_subscription(
            PointCloud2, "/gensong/depth_pcl", self.cb, 10
        )
        self.cloud_cache = None
        self.frame_id = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info("等待 /gensong/depth_pcl 数据...")

    def cb(self, msg: PointCloud2):
        try:
            self.frame_id = msg.header.frame_id
            self.get_logger().info(f"收到点云，frame_id = '{self.frame_id}'")

            pts = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
            if len(pts) == 0:
                self.get_logger().warn("收到空点云")
                return
            xyz = np.column_stack([pts["x"].astype(np.float64), pts["y"].astype(np.float64), pts["z"].astype(np.float64)])
            # 过滤 NaN/Inf
            mask = np.all(np.isfinite(xyz), axis=1)
            self.cloud_cache = xyz[mask]
            n = len(self.cloud_cache)
            self.get_logger().info(
                f"收到 {n} 个点, "
                f"X [{self.cloud_cache[:,0].min():.3f}, {self.cloud_cache[:,0].max():.3f}], "
                f"Y [{self.cloud_cache[:,1].min():.3f}, {self.cloud_cache[:,1].max():.3f}], "
                f"Z [{self.cloud_cache[:,2].min():.3f}, {self.cloud_cache[:,2].max():.3f}]"
            )
        except Exception as e:
            self.get_logger().error(f"解析点云失败: {e}")


def voxelize(cloud, voxel_size=0.02):
    """体素化降采样：每个 voxel 内保留一个点（取均值）。"""
    if len(cloud) == 0:
        return cloud
    idx = np.floor(cloud / voxel_size).astype(np.int64)
    # 每个 voxel 分配唯一 key
    keys = idx[:, 0] * 1000003 + idx[:, 1] * 10007 + idx[:, 2]
    _, inv, counts = np.unique(keys, return_inverse=True, return_counts=True)
    # 按 key 分组求和再平均
    summed = np.zeros((len(counts), 3), dtype=np.float64)
    np.add.at(summed, inv, cloud)
    return summed / counts[:, None]


def transform_to_world(cloud, source_frame, tf_buffer):
    """通过 TF 将点云从 source_frame 转换到 world 坐标系。"""
    from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
    try:
        tf = tf_buffer.lookup_transform("world", source_frame, rclpy.time.Time())
        tx, ty, tz = tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z
        qx, qy, qz, qw = tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w
        # 旋转矩阵从四元数
        R = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)],
        ])
        cloud_world = cloud @ R.T + np.array([tx, ty, tz])
        print(f"TF {source_frame} → world: trans=({tx:.3f}, {ty:.3f}, {tz:.3f}) rot_quat=({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})")
        return cloud_world
    except (LookupException, ConnectivityException, ExtrapolationException) as e:
        print(f"TF 转换失败: {e}")
        return cloud


def plot_3d(cloud, voxel_size=0.02, title="Depth Camera Point Cloud"):
    """绘制体素化 3D 散点图，颜色按 Z 值映射。"""
    cloud_vx = voxelize(cloud, voxel_size)
    print(f"体素化: {len(cloud)} → {len(cloud_vx)} 个点 (voxel={voxel_size}m)")
    x, y, z = cloud_vx[:, 0], cloud_vx[:, 1], cloud_vx[:, 2]

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    scatter = ax.scatter(x, y, z, c=z, cmap="jet", s=0.5, alpha=0.6)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_title(title)
    cbar = fig.colorbar(scatter, ax=ax, shrink=0.6)
    cbar.set_label("Z [m]")

    # 等比例缩放
    max_range = max(x.max() - x.min(), y.max() - y.min(), z.max() - z.min()) / 2.0
    mid_x, mid_y, mid_z = (x.max() + x.min()) / 2, (y.max() + y.min()) / 2, (z.max() + z.min()) / 2
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(description="深度点云 3D 可视化")
    parser.add_argument("--world", action="store_true", help="通过 TF 转换到 world 坐标系")
    parser.add_argument("--voxel", type=float, default=0.02, help="体素大小 (m)")
    args = parser.parse_args()

    import time
    rclpy.init()
    viewer = DepthPCL3DViewer()

    # 收集数据：spin 最多 5 秒收一帧
    deadline = time.time() + 5.0
    while rclpy.ok() and time.time() < deadline:
        rclpy.spin_once(viewer, timeout_sec=0.1)
        if viewer.cloud_cache is not None:
            break

    frame_id = viewer.frame_id
    cloud = viewer.cloud_cache
    viewer.destroy_node()
    rclpy.shutdown()

    if cloud is None:
        print("未收到点云数据，请确保 Isaac Sim 正在运行。")
        sys.exit(1)

    print(f"点云 frame_id = '{frame_id}'")
    if args.world:
        if frame_id and frame_id != "world":
            rclpy.init()
            tf_node = Node("tf_helper")
            tf_buffer = Buffer()
            tf_listener = TransformListener(tf_buffer, tf_node)
            time.sleep(1.0)  # 等 TF 缓存
            cloud = transform_to_world(cloud, frame_id, tf_buffer)
            tf_node.destroy_node()
            rclpy.shutdown()
        title = f"Point Cloud in world frame (from '{frame_id}')"
    else:
        title = f"Point Cloud in '{frame_id}' frame"

    print(f"绘制 {len(cloud)} 个点...")
    plot_3d(cloud, voxel_size=args.voxel, title=title)


if __name__ == "__main__":
    main()