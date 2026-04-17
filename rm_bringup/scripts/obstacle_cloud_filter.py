#!/usr/bin/env python3
"""
obstacle_cloud_filter.py — 从 /cloud_registered 提取障碍物点云给 Nav2

输入: /cloud_registered (PointCloud2, odom 系)
输出: /nav_obstacle_cloud (PointCloud2, odom 系)

过滤: 高度裁剪(去地面) + 距离裁剪 + 体素降采样
"""
import struct
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


class ObstacleCloudFilter(Node):
    def __init__(self):
        super().__init__('obstacle_cloud_filter')
        self.declare_parameter('input_topic', '/cloud_registered')
        self.declare_parameter('output_topic', '/nav_obstacle_cloud')
        self.declare_parameter('z_min', 0.10)
        self.declare_parameter('z_max', 1.50)
        self.declare_parameter('min_range', 0.20)
        self.declare_parameter('max_range', 5.00)
        self.declare_parameter('voxel_size', 0.10)

        self.z_min = self.get_parameter('z_min').value
        self.z_max = self.get_parameter('z_max').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.voxel_size = self.get_parameter('voxel_size').value

        self.pub = self.create_publisher(
            PointCloud2, self.get_parameter('output_topic').value, 5)
        self.sub = self.create_subscription(
            PointCloud2, self.get_parameter('input_topic').value,
            self.cloud_callback, 5)

        self.get_logger().info(
            f'Obstacle filter: z=[{self.z_min},{self.z_max}] '
            f'range=[{self.min_range},{self.max_range}] voxel={self.voxel_size}')

    def cloud_callback(self, msg: PointCloud2):
        offset_x = offset_y = offset_z = None
        point_step = msg.point_step
        for f in msg.fields:
            if f.name == 'x': offset_x = f.offset
            elif f.name == 'y': offset_y = f.offset
            elif f.name == 'z': offset_z = f.offset
        if offset_x is None or offset_y is None or offset_z is None:
            return

        n_points = msg.width * msg.height
        if n_points == 0:
            return

        raw = np.frombuffer(msg.data, dtype=np.uint8)
        xs = np.zeros(n_points, dtype=np.float32)
        ys = np.zeros(n_points, dtype=np.float32)
        zs = np.zeros(n_points, dtype=np.float32)

        for i in range(n_points):
            base = i * point_step
            xs[i] = struct.unpack_from('f', raw, base + offset_x)[0]
            ys[i] = struct.unpack_from('f', raw, base + offset_y)[0]
            zs[i] = struct.unpack_from('f', raw, base + offset_z)[0]

        valid = np.isfinite(xs) & np.isfinite(ys) & np.isfinite(zs)
        xs, ys, zs = xs[valid], ys[valid], zs[valid]
        if len(xs) == 0:
            return

        mask_z = (zs > self.z_min) & (zs < self.z_max)
        ranges = np.sqrt(xs**2 + ys**2 + zs**2)
        mask_r = (ranges > self.min_range) & (ranges < self.max_range)
        mask = mask_z & mask_r
        xs, ys, zs = xs[mask], ys[mask], zs[mask]
        if len(xs) == 0:
            return

        if self.voxel_size > 0:
            vi = np.floor(xs / self.voxel_size).astype(np.int32)
            vj = np.floor(ys / self.voxel_size).astype(np.int32)
            vk = np.floor(zs / self.voxel_size).astype(np.int32)
            seen = {}
            indices = []
            for idx in range(len(xs)):
                key = (vi[idx], vj[idx], vk[idx])
                if key not in seen:
                    seen[key] = True
                    indices.append(idx)
            xs, ys, zs = xs[indices], ys[indices], zs[indices]

        if len(xs) == 0:
            return

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = len(xs)
        out.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        out.is_bigendian = False
        out.point_step = 12
        out.row_step = 12 * len(xs)
        out.is_dense = True

        buf = bytearray(out.row_step)
        for i in range(len(xs)):
            struct.pack_into('fff', buf, i * 12, float(xs[i]), float(ys[i]), float(zs[i]))
        out.data = bytes(buf)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ObstacleCloudFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
