#!/usr/bin/env python3
"""
handeye_capture.py — ROS 2 手眼标定数据采集工具

功能:
  - 订阅 /camera/image_raw + /imu/data
  - 实时显示图像 + 棋盘格检测结果 + IMU 欧拉角
  - 按 's' 保存同步的 (图像, IMU四元数) 对
  - 按 'q' 退出

用法:
  终端1: ros2 launch rm_bringup sentry_bringup.launch.py   (或分别启动相机+hw_bridge)
  终端2: python3 tools/handeye_capture.py

标定板: 8×11 内角点, 方格边长 15mm (可通过参数覆盖)
"""

import os
import sys
import json
import math
import argparse

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class HandEyeCapture(Node):
    def __init__(self, output_dir, pattern_cols, pattern_rows, square_mm):
        super().__init__('handeye_capture')

        self.output_dir = output_dir
        self.pattern_size = (pattern_cols, pattern_rows)
        self.square_mm = square_mm
        self.bridge = CvBridge()

        # 最新 IMU 四元数 (w, x, y, z)
        self.latest_q = None
        self.latest_euler = (0.0, 0.0, 0.0)  # (yaw, pitch, roll) deg

        # 最新图像
        self.latest_img = None
        self.latest_stamp = None

        # QoS: BEST_EFFORT 匹配相机驱动
        qos_best = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # 订阅
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, qos_best
        )
        self.img_sub = self.create_subscription(
            Image, '/camera/image_raw', self.img_callback, qos_best
        )

        self.count = 0
        os.makedirs(self.output_dir, exist_ok=True)

        self.get_logger().info(
            f'手眼标定采集启动 | 标定板: {pattern_cols}x{pattern_rows} 内角点, '
            f'方格={square_mm}mm | 输出: {output_dir}'
        )
        self.get_logger().info("按 's' 保存, 'q' 退出")

    # -------------------------------------------------------------------
    # IMU 四元数 → 欧拉角 (ZYX, 即 yaw-pitch-roll)
    # -------------------------------------------------------------------
    @staticmethod
    def quat_to_euler(w, x, y, z):
        """四元数 → (yaw, pitch, roll) in degrees"""
        # roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = 2.0 * (w * y - z * x)
        sinp = max(-1.0, min(1.0, sinp))
        pitch = math.asin(sinp)

        # yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return (
            math.degrees(yaw),
            math.degrees(pitch),
            math.degrees(roll),
        )

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        self.latest_q = (q.w, q.x, q.y, q.z)
        self.latest_euler = self.quat_to_euler(q.w, q.x, q.y, q.z)

    def img_callback(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.latest_img = img
        self.latest_stamp = msg.header.stamp

        # 检测棋盘格
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCornersSB(gray, self.pattern_size)

        # 精炼
        if found:
            corners = cv2.cornerSubPix(
                gray, corners, (5, 5), (-1, -1),
                criteria=(cv2.TermCriteria_EPS + cv2.TermCriteria_MAX_ITER, 30, 1e-3),
            )

        # 绘制
        display = img.copy()
        if found:
            cv2.drawChessboardCorners(display, self.pattern_size, corners, found)

        # 叠加 IMU 信息
        yaw, pitch, roll = self.latest_euler
        info_lines = [
            f"IMU: yaw={yaw:+.1f} pitch={pitch:+.1f} roll={roll:+.1f}",
            f"Chessboard: {'DETECTED' if found else 'NOT FOUND'}",
            f"Saved: {self.count} | Press 's' to save, 'q' to quit",
        ]
        for i, line in enumerate(info_lines):
            color = (0, 255, 0) if found and i == 1 else (0, 0, 255)
            cv2.putText(display, line, (20, 40 + i * 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        # 缩放显示
        h, w = display.shape[:2]
        scale = min(1280 / w, 720 / h)
        display = cv2.resize(display, (int(w * scale), int(h * scale)))

        cv2.imshow("HandEye Capture", display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('s') and found and self.latest_q is not None:
            self.count += 1
            # 保存图像
            img_path = os.path.join(self.output_dir, f"{self.count}.jpg")
            cv2.imwrite(img_path, img)

            # 保存四元数 (w x y z) + 时间戳
            meta = {
                "w": self.latest_q[0],
                "x": self.latest_q[1],
                "y": self.latest_q[2],
                "z": self.latest_q[3],
                "stamp_sec": self.latest_stamp.sec,
                "stamp_nsec": self.latest_stamp.nanosec,
            }
            q_path = os.path.join(self.output_dir, f"{self.count}.json")
            with open(q_path, 'w') as f:
                json.dump(meta, f, indent=2)

            self.get_logger().info(
                f"[{self.count}] Saved | yaw={yaw:+.1f} pitch={pitch:+.1f} roll={roll:+.1f}"
            )

        elif key == ord('s') and (not found or self.latest_q is None):
            reason = "no IMU data" if self.latest_q is None else "chessboard not detected"
            self.get_logger().warn(f"Skipped: {reason}")

        elif key == ord('q'):
            self.get_logger().info(f"退出。共保存 {self.count} 组数据到 {self.output_dir}")
            raise SystemExit


def main():
    parser = argparse.ArgumentParser(description='手眼标定数据采集')
    parser.add_argument('-o', '--output', default='handeye_data',
                        help='输出目录 (默认: handeye_data)')
    parser.add_argument('--cols', type=int, default=11,
                        help='标定板内角点列数 (默认: 11)')
    parser.add_argument('--rows', type=int, default=8,
                        help='标定板内角点行数 (默认: 8)')
    parser.add_argument('--square', type=float, default=15.0,
                        help='方格边长 mm (默认: 15)')
    args = parser.parse_args()

    rclpy.init()
    node = HandEyeCapture(args.output, args.cols, args.rows, args.square)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass

    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()