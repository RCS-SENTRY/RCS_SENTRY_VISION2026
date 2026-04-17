#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSPresetProfiles
from rm_interfaces.msg import NavCmd


class CmdVelToNavCmd(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_to_nav_cmd")

        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter("output_topic", "/nav_cmd")
        self.declare_parameter("cmd_vel_timeout_sec", 0.25)
        self.declare_parameter("publish_rate_hz", 20.0)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.cmd_vel_timeout_sec = float(self.get_parameter("cmd_vel_timeout_sec").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if publish_rate_hz <= 1e-6:
            publish_rate_hz = 20.0

        self.nav_cmd_pub = self.create_publisher(
            NavCmd, output_topic, QoSPresetProfiles.SENSOR_DATA.value
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist, input_topic, self.on_cmd_vel, 10
        )
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.on_timer)

        self.latest_cmd = Twist()
        self.latest_cmd_time = self.get_clock().now()
        self.has_cmd = False
        self.timeout_reported = False

        self.get_logger().info(
            "cmd_vel_to_nav_cmd ready: %s -> %s, timeout=%.2fs"
            % (input_topic, output_topic, self.cmd_vel_timeout_sec)
        )

    def on_cmd_vel(self, msg: Twist) -> None:
        self.latest_cmd = msg
        self.latest_cmd_time = self.get_clock().now()
        self.has_cmd = True
        self.timeout_reported = False
        self.nav_cmd_pub.publish(self.twist_to_nav_cmd(msg))

    def on_timer(self) -> None:
        if not self.has_cmd:
            return

        age = (self.get_clock().now() - self.latest_cmd_time).nanoseconds / 1e9
        if age <= self.cmd_vel_timeout_sec:
            self.nav_cmd_pub.publish(self.twist_to_nav_cmd(self.latest_cmd))
            return

        self.nav_cmd_pub.publish(self.twist_to_nav_cmd(Twist()))
        if not self.timeout_reported:
            self.timeout_reported = True
            self.get_logger().warn(
                "cmd_vel timeout %.3fs > %.3fs, publishing continuous zero /nav_cmd"
                % (age, self.cmd_vel_timeout_sec)
            )

    @staticmethod
    def twist_to_nav_cmd(msg: Twist) -> NavCmd:
        nav = NavCmd()
        nav.linear_x = float(msg.linear.x)
        nav.linear_y = float(msg.linear.y)
        nav.angular_z = float(msg.angular.z)
        # 这里只做速度桥接，不擅自写入高层任务状态。
        # 后续如果需要和 NavigateToPose action 结果联动，再单独补 is_reached。
        nav.is_reached = 0
        return nav


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelToNavCmd()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # 退出前补一帧零速，避免控制链残留旧命令
        try:
            node.nav_cmd_pub.publish(CmdVelToNavCmd.twist_to_nav_cmd(Twist()))
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
