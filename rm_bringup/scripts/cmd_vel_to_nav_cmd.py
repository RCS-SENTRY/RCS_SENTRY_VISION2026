#!/usr/bin/env python3

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rm_interfaces.msg import NavCmd
from std_msgs.msg import String


class CmdVelToNavCmd(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_to_nav_cmd")

        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter("output_topic", "/nav_cmd")
        self.declare_parameter("cmd_vel_timeout_sec", 0.25)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter(
            "navigate_status_topic", "/navigate_to_pose/_action/status"
        )
        self.declare_parameter("goal_reached_latch_sec", 1.0)
        self.declare_parameter(
            "recovery_status_topic", "/localization_recovery_status"
        )

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        navigate_status_topic = str(self.get_parameter("navigate_status_topic").value)
        recovery_status_topic = str(self.get_parameter("recovery_status_topic").value)
        self.cmd_vel_timeout_sec = float(self.get_parameter("cmd_vel_timeout_sec").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.goal_reached_latch_sec = float(
            self.get_parameter("goal_reached_latch_sec").value
        )
        if publish_rate_hz <= 1e-6:
            publish_rate_hz = 20.0
        if self.goal_reached_latch_sec < 0.0:
            self.goal_reached_latch_sec = 0.0

        self.nav_cmd_pub = self.create_publisher(
            NavCmd, output_topic, QoSPresetProfiles.SENSOR_DATA.value
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist, input_topic, self.on_cmd_vel, 10
        )
        self.goal_status_sub = self.create_subscription(
            GoalStatusArray,
            navigate_status_topic,
            self.on_goal_status,
            10,
        )
        self.recovery_status_sub = self.create_subscription(
            String,
            recovery_status_topic,
            self.on_recovery_status,
            10,
        )
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.on_timer)

        self.latest_cmd = Twist()
        self.latest_cmd_time = self.get_clock().now()
        self.goal_reached_until = None
        self.recovery_state = "WAITING_FOR_INITIALPOSE"
        self.has_cmd = False
        self.timeout_reported = False

        self.get_logger().info(
            "cmd_vel_to_nav_cmd ready: %s -> %s, timeout=%.2fs, status=%s, recovery=%s"
            % (
                input_topic,
                output_topic,
                self.cmd_vel_timeout_sec,
                navigate_status_topic,
                recovery_status_topic,
            )
        )

    def on_cmd_vel(self, msg: Twist) -> None:
        self.latest_cmd = msg
        self.latest_cmd_time = self.get_clock().now()
        self.has_cmd = True
        self.timeout_reported = False
        if self.recovery_state not in ("LOST", "RECOVERING"):
            self.nav_cmd_pub.publish(self.twist_to_nav_cmd(msg, is_reached=0))

    def on_goal_status(self, msg: GoalStatusArray) -> None:
        if not msg.status_list:
            return

        latest_status = msg.status_list[-1].status
        if latest_status == GoalStatus.STATUS_SUCCEEDED:
            now = self.get_clock().now()
            self.goal_reached_until = now + Duration(seconds=self.goal_reached_latch_sec)
            self.nav_cmd_pub.publish(self.twist_to_nav_cmd(Twist(), is_reached=1))
            self.get_logger().info("NavigateToPose succeeded, latching nav_cmd.is_reached=1")
        elif latest_status in (
            GoalStatus.STATUS_ACCEPTED,
            GoalStatus.STATUS_EXECUTING,
            GoalStatus.STATUS_CANCELING,
        ):
            self.goal_reached_until = None
        elif latest_status in (GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED):
            self.goal_reached_until = None

    def on_recovery_status(self, msg: String) -> None:
        self.recovery_state = msg.data

    def on_timer(self) -> None:
        if self.recovery_state in ("LOST", "RECOVERING"):
            self.goal_reached_until = None
            self.nav_cmd_pub.publish(self.twist_to_nav_cmd(Twist(), is_reached=0))
            return

        if self.goal_reached_until is not None:
            if self.get_clock().now() <= self.goal_reached_until:
                self.nav_cmd_pub.publish(self.twist_to_nav_cmd(Twist(), is_reached=1))
                return
            self.goal_reached_until = None

        if not self.has_cmd:
            return

        age = (self.get_clock().now() - self.latest_cmd_time).nanoseconds / 1e9
        if age <= self.cmd_vel_timeout_sec:
            self.nav_cmd_pub.publish(self.twist_to_nav_cmd(self.latest_cmd, is_reached=0))
            return

        self.nav_cmd_pub.publish(self.twist_to_nav_cmd(Twist(), is_reached=0))
        if not self.timeout_reported:
            self.timeout_reported = True
            self.get_logger().warn(
                "cmd_vel timeout %.3fs > %.3fs, publishing continuous zero /nav_cmd"
                % (age, self.cmd_vel_timeout_sec)
            )

    @staticmethod
    def twist_to_nav_cmd(msg: Twist, is_reached: int = 0) -> NavCmd:
        nav = NavCmd()
        nav.linear_x = float(msg.linear.x)
        nav.linear_y = float(msg.linear.y)
        nav.angular_z = float(msg.angular.z)
        nav.is_reached = int(is_reached)
        return nav


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelToNavCmd()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.nav_cmd_pub.publish(CmdVelToNavCmd.twist_to_nav_cmd(Twist(), is_reached=0))
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
