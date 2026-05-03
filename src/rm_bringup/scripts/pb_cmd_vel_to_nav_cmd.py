#!/usr/bin/env python3

import math
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rm_interfaces.msg import NavCmd


class PbCmdVelToNavCmd(Node):
    def __init__(self) -> None:
        super().__init__("pb_cmd_vel_to_nav_cmd")

        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter("output_topic", "/nav_cmd")
        self.declare_parameter("cmd_vel_timeout_sec", 0.25)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("navigate_to_pose_status_topic", "/navigate_to_pose/_action/status")
        self.declare_parameter(
            "navigate_through_poses_status_topic",
            "/navigate_through_poses/_action/status",
        )
        self.declare_parameter("goal_reached_latch_sec", 1.0)
        self.declare_parameter("force_zero_angular_z", True)
        self.declare_parameter("invert_linear_y", False)
        self.declare_parameter("shutdown_zero_burst_count", 8)
        self.declare_parameter("shutdown_zero_burst_interval_sec", 0.02)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        navigate_to_pose_status_topic = str(
            self.get_parameter("navigate_to_pose_status_topic").value
        )
        navigate_through_poses_status_topic = str(
            self.get_parameter("navigate_through_poses_status_topic").value
        )
        self.cmd_vel_timeout_sec = float(self.get_parameter("cmd_vel_timeout_sec").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.goal_reached_latch_sec = float(
            self.get_parameter("goal_reached_latch_sec").value
        )
        self.force_zero_angular_z = bool(
            self.get_parameter("force_zero_angular_z").value
        )
        self.invert_linear_y = bool(self.get_parameter("invert_linear_y").value)
        self.shutdown_zero_burst_count = int(
            self.get_parameter("shutdown_zero_burst_count").value
        )
        self.shutdown_zero_burst_interval_sec = float(
            self.get_parameter("shutdown_zero_burst_interval_sec").value
        )

        if publish_rate_hz <= 1e-6 or not math.isfinite(publish_rate_hz):
            publish_rate_hz = 20.0
        if self.goal_reached_latch_sec < 0.0:
            self.goal_reached_latch_sec = 0.0
        if self.shutdown_zero_burst_count < 1:
            self.shutdown_zero_burst_count = 1
        if (
            self.shutdown_zero_burst_interval_sec < 0.0
            or not math.isfinite(self.shutdown_zero_burst_interval_sec)
        ):
            self.shutdown_zero_burst_interval_sec = 0.02

        self.nav_cmd_pub = self.create_publisher(
            NavCmd, output_topic, QoSPresetProfiles.SENSOR_DATA.value
        )
        self.cmd_vel_sub = self.create_subscription(Twist, input_topic, self.on_cmd_vel, 10)
        self.nav_to_pose_status_sub = self.create_subscription(
            GoalStatusArray,
            navigate_to_pose_status_topic,
            self.on_goal_status,
            10,
        )
        self.nav_through_poses_status_sub = self.create_subscription(
            GoalStatusArray,
            navigate_through_poses_status_topic,
            self.on_goal_status,
            10,
        )
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.on_timer)

        self.latest_cmd = Twist()
        self.latest_cmd_time = self.get_clock().now()
        self.goal_reached_until = None
        self.has_cmd = False
        self.timeout_reported = False

        self.get_logger().info(
            "PB cmd_vel bridge ready: %s -> %s, timeout=%.2fs, force_zero_angular_z=%s"
            % (
                input_topic,
                output_topic,
                self.cmd_vel_timeout_sec,
                str(self.force_zero_angular_z).lower(),
            )
        )

    def on_cmd_vel(self, msg: Twist) -> None:
        self.latest_cmd = msg
        self.latest_cmd_time = self.get_clock().now()
        self.has_cmd = True
        self.timeout_reported = False
        self.nav_cmd_pub.publish(self.twist_to_nav_cmd(msg, is_reached=0))

    def on_goal_status(self, msg: GoalStatusArray) -> None:
        if not msg.status_list:
            return

        latest_status = msg.status_list[-1].status
        if latest_status == GoalStatus.STATUS_SUCCEEDED:
            now = self.get_clock().now()
            self.goal_reached_until = now + Duration(seconds=self.goal_reached_latch_sec)
            self.nav_cmd_pub.publish(self.twist_to_nav_cmd(Twist(), is_reached=1))
        elif latest_status in (
            GoalStatus.STATUS_ACCEPTED,
            GoalStatus.STATUS_EXECUTING,
            GoalStatus.STATUS_CANCELING,
            GoalStatus.STATUS_ABORTED,
            GoalStatus.STATUS_CANCELED,
        ):
            self.goal_reached_until = None

    def on_timer(self) -> None:
        now = self.get_clock().now()

        if self.goal_reached_until is not None:
            if now <= self.goal_reached_until:
                self.nav_cmd_pub.publish(self.twist_to_nav_cmd(Twist(), is_reached=1))
                return
            self.goal_reached_until = None

        if not self.has_cmd:
            self.nav_cmd_pub.publish(self.twist_to_nav_cmd(Twist(), is_reached=0))
            return

        age = (now - self.latest_cmd_time).nanoseconds / 1e9
        if age <= self.cmd_vel_timeout_sec:
            self.nav_cmd_pub.publish(self.twist_to_nav_cmd(self.latest_cmd, is_reached=0))
            return

        self.nav_cmd_pub.publish(self.twist_to_nav_cmd(Twist(), is_reached=0))
        if not self.timeout_reported:
            self.timeout_reported = True
            self.get_logger().warn(
                "PB cmd_vel timeout %.3fs > %.3fs; publishing zero NavCmd"
                % (age, self.cmd_vel_timeout_sec)
            )

    def twist_to_nav_cmd(self, msg: Twist, is_reached: int = 0) -> NavCmd:
        nav = NavCmd()
        nav.linear_x = float(msg.linear.x)
        nav.linear_y = -float(msg.linear.y) if self.invert_linear_y else float(msg.linear.y)
        nav.angular_z = 0.0 if self.force_zero_angular_z else float(msg.angular.z)
        nav.is_reached = int(is_reached)
        return nav

    def publish_shutdown_zero_burst(self) -> None:
        zero = self.twist_to_nav_cmd(Twist(), is_reached=0)
        for _ in range(self.shutdown_zero_burst_count):
            self.nav_cmd_pub.publish(zero)
            try:
                rclpy.spin_once(self, timeout_sec=0.0)
            except Exception:
                pass
            time.sleep(self.shutdown_zero_burst_interval_sec)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PbCmdVelToNavCmd()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.get_logger().info("Shutdown requested; publishing zero NavCmd burst")
            node.publish_shutdown_zero_burst()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
