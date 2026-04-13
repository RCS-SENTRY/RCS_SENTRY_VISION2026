#!/usr/bin/env python3
"""
fake_sensor_publisher.py — 无下位机时的假传感器发布器 (闭环版)

闭环逻辑:
  订阅 /gimbal_cmd → 模拟云台跟踪 → 发布 GimbalStatus (角度回传)
  这样 Gate 1 (炮管对齐) 就能正常通过。

用法:
  终端1: ros2 bag play ... --topics /gimbal/imu /detector/armors --clock --loop
  终端2: python3 tools/fake_sensor_publisher.py   ← 替代 bag 中的 /gimbal/status
  终端3: ros2 run rm_autoaim autoaim_node ...
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
from rm_interfaces.msg import GimbalStatus, GimbalCmd


class FakeSensorPublisher(Node):
    def __init__(self):
        super().__init__('fake_sensor_publisher')

        # ---- 参数 ----
        self.declare_parameter('imu_rate', 500)
        self.declare_parameter('status_rate', 100)
        self.declare_parameter('fake_bullet_speed', 15.0)
        self.declare_parameter('track_gain', 0.8)  # 云台跟踪增益 (1.0=完美)

        imu_rate = self.get_parameter('imu_rate').value
        status_rate = self.get_parameter('status_rate').value
        self.fake_bullet = self.get_parameter('fake_bullet_speed').value
        self.track_gain = self.get_parameter('track_gain').value

        # ---- 闭环状态 ----
        self.current_yaw = 0.0      # 当前模拟云台 yaw (rad)
        self.current_pitch = 0.0    # 当前模拟云台 pitch (rad)
        self.target_yaw = 0.0
        self.target_pitch = 0.0
        self.has_cmd = False

        # ---- QoS ----
        qos_best = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ---- Publisher ----
        self.imu_pub = self.create_publisher(Imu, '/gimbal/imu', qos_best)
        self.status_pub = self.create_publisher(GimbalStatus, '/gimbal/status', qos_best)

        # ---- Subscriber (闭环核心) ----
        self.cmd_sub = self.create_subscription(
            GimbalCmd, '/gimbal_cmd', self.cmd_callback, qos_reliable)

        # ---- Timer ----
        self.imu_timer = self.create_timer(1.0 / imu_rate, self.publish_imu)
        self.status_timer = self.create_timer(1.0 / status_rate, self.publish_status)

        self.imu_count = 0
        self.status_count = 0

        self.get_logger().info(
            f'Fake sensor (闭环) started. '
            f'IMU={imu_rate}Hz, Status={status_rate}Hz, '
            f'bullet={self.fake_bullet}m/s, gain={self.track_gain}'
        )

    def cmd_callback(self, msg):
        """接收 gimbal_cmd, 记录目标角度"""
        self.target_yaw = msg.target_yaw
        self.target_pitch = msg.target_pitch
        self.has_cmd = True

    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gimbal_imu'

        # 单位四元数
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0

        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 0.0

        self.imu_pub.publish(msg)
        self.imu_count += 1

    def publish_status(self):
        """闭环: 将 current 角度向 target 角度渐进跟踪"""
        if self.has_cmd:
            # 一阶跟踪: current += gain * (target - current)
            self.current_yaw += self.track_gain * (self.target_yaw - self.current_yaw)
            self.current_pitch += self.track_gain * (self.target_pitch - self.current_pitch)

        msg = GimbalStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gimbal_status'

        # GimbalStatus 角度单位: deg
        # aimer.hpp 中 gateBarrelAlignment 从 GimbalStatus 读取的是 deg
        # GimbalCmd 的 target_yaw/pitch 单位是 rad
        # 所这里要转成 deg
        msg.gimbal_yaw = self.current_yaw * 180.0 / 3.14159265
        msg.gimbal_pitch = self.current_pitch * 180.0 / 3.14159265
        msg.gimbal_yaw_rate = 0.0
        msg.gimbal_pitch_rate = 0.0
        msg.bullet_speed = float(self.fake_bullet)
        msg.mode = 1  # auto_aim

        self.status_pub.publish(msg)
        self.status_count += 1

        if self.status_count % 100 == 0:
            self.get_logger().info(
                f'Status: {self.status_count} msgs | '
                f'yaw={msg.gimbal_yaw:.1f}° pitch={msg.gimbal_pitch:.1f}° | '
                f'cmd_yaw={self.target_yaw*180/3.14159:.1f}°'
            )


def main():
    rclpy.init()
    node = FakeSensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()