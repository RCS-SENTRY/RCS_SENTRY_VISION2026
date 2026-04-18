#!/usr/bin/env python3
"""
fake_lidar_publisher.py — 模拟 Livox MID-360 点云数据（无雷达测试用）

发布:
  /livox/lidar           (CustomMsg) — 模拟 Livox 原始 CustomMsg
  /livox/lidar/pointcloud (PointCloud2) — 模拟 Livox 原始 PointCloud2

点云内容:
  - 环境点: 地面 + 四面墙壁 + 随机障碍物
  - 自车点: 车体 + 云台 + 支撑结构 + 雷达臂 (应该被 self_point_filter 剔除)
  - 右侧额外点云: 模拟实际右侧云台结构回波 (应该被剔除)

用法:
  ros2 run rm_bringup fake_lidar_publisher.py
"""

import math
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from livox_ros_driver2.msg import CustomMsg, CustomPoint


class FakeLidarPublisher(Node):
    def __init__(self):
        super().__init__('fake_lidar_publisher')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub_custom = self.create_publisher(CustomMsg, '/livox/lidar', qos)
        self.pub_pc2 = self.create_publisher(PointCloud2, '/livox/lidar/pointcloud', qos)

        self.frame_id = 'livox_frame'
        self.timer = self.create_timer(0.05, self.publish_fake_cloud)  # 20Hz
        self.msg_count = 0

        # TF: base_link -> livox_frame (from launch params)
        # lidar_x=0.0, lidar_y=0.2, lidar_z=0.35, pitch=0.3115, yaw=1.5708
        # For simplicity, we generate points in livox_frame directly

        self.get_logger().info('Fake LiDAR publisher started (20Hz, livox_frame)')

    def publish_fake_cloud(self):
        now_ns = self.get_clock().now().nanoseconds
        now_sec = self.get_clock().now().seconds_nanoseconds()

        points_livox = []  # (x, y, z) in livox_frame

        # ---- 1. 环境点 (livox_frame, 模拟房间 ~6m x 6m) ----
        # 地面 (z ≈ -0.35 in livox_frame, because lidar is at z=0.35 in base_link)
        for x in self._frange(-3.0, 3.0, 0.3):
            for y in self._frange(-3.0, 3.0, 0.3):
                points_livox.append((x, y, -0.35))

        # 前墙 (x = 3.0)
        for z in self._frange(-0.35, 1.5, 0.2):
            for y in self._frange(-3.0, 3.0, 0.3):
                points_livox.append((3.0, y, z))

        # 后墙 (x = -3.0)
        for z in self._frange(-0.35, 1.5, 0.2):
            for y in self._frange(-3.0, 3.0, 0.3):
                points_livox.append((-3.0, y, z))

        # 左墙 (y = 3.0)
        for z in self._frange(-0.35, 1.5, 0.2):
            for x in self._frange(-3.0, 3.0, 0.3):
                points_livox.append((x, 3.0, z))

        # 右墙 (y = -3.0)
        for z in self._frange(-0.35, 1.5, 0.2):
            for x in self._frange(-3.0, 3.0, 0.3):
                points_livox.append((x, -3.0, z))

        # 随机障碍物 (前方 1.5m)
        for z in self._frange(-0.2, 0.5, 0.1):
            for y in self._frange(-0.3, 0.3, 0.1):
                points_livox.append((1.5, y, z))

        # ---- 2. 自车点云 (这些应该在 base_link 中被 exclusion boxes 剔除) ----
        # 注意: 这些点是 livox_frame 坐标系下的
        # base_link 原点在 lidar 下方 0.35m, 偏左 0.2m, 并且有旋转
        # 为了简单, 我们直接在 base_link 坐标系生成自车点, 然后变换到 livox_frame

        # 简化 TF 变换: livox -> base_link 的逆
        # 实际 TF: x=0, y=0.2, z=0.35, roll=0, pitch=0.3115, yaw=pi/2
        # 这里我们直接在 livox_frame 里模拟一些"近距离回波"点
        # 这些点在 base_link 中会落入 exclusion boxes

        # 车体表面回波 (近距离, 在 livox_frame 坐标系中模拟)
        # lidar 在车体上方, 所以向下/向后扫到的车体点大约在:
        # livox_frame 中 x~0(后向), y~[-0.2, 0.2](左右), z~[-0.35, -0.1](下方)
        for x in self._frange(-0.15, 0.15, 0.05):
            for y in self._frange(-0.15, 0.25, 0.05):
                points_livox.append((x, y, -0.30))  # 车体顶面回波

        # 云台回波 (更高一点的结构)
        for x in self._frange(-0.05, 0.10, 0.03):
            for y in self._frange(-0.05, 0.15, 0.03):
                points_livox.append((x, y, -0.10))  # 云台结构

        # 右侧遮挡结构回波 (lidar 偏右安装, y=+0.2处有结构)
        for z in self._frange(-0.30, -0.05, 0.05):
            for y in self._frange(0.10, 0.25, 0.03):
                points_livox.append((-0.05, y, z))  # 右侧遮挡

        # 雷达安装臂
        for z in self._frange(-0.25, -0.10, 0.05):
            points_livox.append((0.0, 0.15, z))
            points_livox.append((0.05, 0.20, z))

        # ---- 发布 CustomMsg ----
        custom_msg = CustomMsg()
        custom_msg.header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id=self.frame_id,
        )
        custom_msg.timebase = now_ns
        custom_msg.lidar_id = 0
        custom_msg.rsvd = [0, 0, 0]

        for i, (x, y, z) in enumerate(points_livox):
            pt = CustomPoint()
            pt.x = x
            pt.y = y
            pt.z = z
            pt.offset_time = i * 1000  # fake offset in ns
            pt.reflectivity = 128
            pt.tag = 0
            pt.line = i % 4
            custom_msg.points.append(pt)

        custom_msg.point_num = len(custom_msg.points)
        self.pub_custom.publish(custom_msg)

        # ---- 发布 PointCloud2 ----
        pc2_msg = PointCloud2()
        pc2_msg.header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id=self.frame_id,
        )
        pc2_msg.height = 1
        pc2_msg.width = len(points_livox)
        pc2_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        pc2_msg.is_bigendian = False
        pc2_msg.point_step = 16
        pc2_msg.row_step = 16 * len(points_livox)
        pc2_msg.is_dense = True

        buf = bytearray()
        for (x, y, z) in points_livox:
            buf.extend(struct.pack('ffff', x, y, z, 128.0))
        pc2_msg.data = bytes(buf)

        self.pub_pc2.publish(pc2_msg)

        self.msg_count += 1
        if self.msg_count % 20 == 0:
            self.get_logger().info(
                f'Published {len(points_livox)} points '
                f'(CustomMsg + PointCloud2), frame #{self.msg_count}'
            )

    @staticmethod
    def _frange(start, stop, step):
        vals = []
        v = start
        while v <= stop + 1e-9:
            vals.append(round(v, 6))
            v += step
        return vals


def main(args=None):
    rclpy.init(args=args)
    node = FakeLidarPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()