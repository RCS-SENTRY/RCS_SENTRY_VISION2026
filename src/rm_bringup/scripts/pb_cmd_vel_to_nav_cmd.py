#!/usr/bin/env python3

import math

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
        self.declare_parameter("enable_heading_align", False)
        self.declare_parameter("heading_align_only_when_force_zero_angular_z", True)
        self.declare_parameter("heading_align_min_speed", 0.15)
        self.declare_parameter("heading_align_kp", 1.2)
        self.declare_parameter("heading_align_max_wz", 0.5)
        self.declare_parameter("heading_align_deadband_rad", 0.08)
        self.declare_parameter("disable_heading_align_when_reached", True)
        self.declare_parameter("invert_linear_y", False)
        self.declare_parameter("max_linear_x", 2.0)
        self.declare_parameter("max_linear_y", 2.0)
        self.declare_parameter("max_linear_speed", 2.0)
        self.declare_parameter("max_angular_z", 0.5)
        self.declare_parameter("max_linear_accel", 2.5)
        self.declare_parameter("max_angular_accel", 1.0)
        self.declare_parameter("log_output_hz", 1.0)

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
        self.force_zero_angular_z = self._bool_param("force_zero_angular_z", True)
        self.enable_heading_align = self._bool_param("enable_heading_align", False)
        self.heading_align_only_when_force_zero_angular_z = self._bool_param(
            "heading_align_only_when_force_zero_angular_z", True
        )
        self.heading_align_min_speed = self._positive_param("heading_align_min_speed", 0.15)
        self.heading_align_kp = self._positive_param("heading_align_kp", 1.2)
        self.heading_align_max_wz = self._positive_param("heading_align_max_wz", 0.5)
        self.heading_align_deadband_rad = self._positive_param(
            "heading_align_deadband_rad", 0.08
        )
        self.disable_heading_align_when_reached = self._bool_param(
            "disable_heading_align_when_reached", True
        )
        self.invert_linear_y = self._bool_param("invert_linear_y", False)
        self.max_linear_x = self._positive_param("max_linear_x", 2.0)
        self.max_linear_y = self._positive_param("max_linear_y", 2.0)
        self.max_linear_speed = self._positive_param("max_linear_speed", 2.0)
        self.max_angular_z = min(
            self._positive_param("max_angular_z", self.heading_align_max_wz),
            self.heading_align_max_wz,
        )
        self.max_linear_accel = self._positive_param("max_linear_accel", 2.5)
        self.max_angular_accel = self._positive_param("max_angular_accel", 1.0)
        self.log_output_period = 1.0 / self._positive_param("log_output_hz", 1.0)

        if publish_rate_hz <= 1e-6 or not math.isfinite(publish_rate_hz):
            publish_rate_hz = 20.0
        if self.goal_reached_latch_sec < 0.0:
            self.goal_reached_latch_sec = 0.0

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
        self.last_publish_time = None
        self.last_nav_cmd = self.twist_to_nav_cmd(Twist(), is_reached=0)[0]
        self.last_clamp_log_time = None
        self.heading_feedback_warned = False

        self.get_logger().info(
            "PB cmd_vel bridge ready: %s -> %s, timeout=%.2fs, "
            "force_zero_angular_z=%s, heading_align=%s, max_xy=(%.2f, %.2f), "
            "max_speed=%.2f, max_wz=%.2f"
            % (
                input_topic,
                output_topic,
                self.cmd_vel_timeout_sec,
                str(self.force_zero_angular_z).lower(),
                str(self.enable_heading_align).lower(),
                self.max_linear_x,
                self.max_linear_y,
                self.max_linear_speed,
                self.max_angular_z,
            )
        )

    def on_cmd_vel(self, msg: Twist) -> None:
        self.latest_cmd = msg
        self.latest_cmd_time = self.get_clock().now()
        self.has_cmd = True
        self.timeout_reported = False

    def on_goal_status(self, msg: GoalStatusArray) -> None:
        if not msg.status_list:
            return

        latest_status = msg.status_list[-1].status
        if latest_status == GoalStatus.STATUS_SUCCEEDED:
            now = self.get_clock().now()
            self.goal_reached_until = now + Duration(seconds=self.goal_reached_latch_sec)
            self.publish_nav_cmd(Twist(), is_reached=1, immediate_stop=True)
            self.get_logger().info(
                "Navigation goal succeeded; publishing is_reached=1 for %.2fs"
                % self.goal_reached_latch_sec
            )
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
                self.publish_nav_cmd(Twist(), is_reached=1, immediate_stop=True)
                return
            self.goal_reached_until = None

        if not self.has_cmd:
            self.publish_nav_cmd(Twist(), is_reached=0, immediate_stop=True)
            return

        age = (now - self.latest_cmd_time).nanoseconds / 1e9
        if age <= self.cmd_vel_timeout_sec:
            self.publish_nav_cmd(self.latest_cmd, is_reached=0)
            return

        self.publish_nav_cmd(Twist(), is_reached=0, immediate_stop=True)
        if not self.timeout_reported:
            self.timeout_reported = True
            self.get_logger().warn(
                "PB cmd_vel timeout %.3fs > %.3fs; publishing zero NavCmd"
                % (age, self.cmd_vel_timeout_sec)
            )

    def publish_nav_cmd(
        self, msg: Twist, is_reached: int = 0, immediate_stop: bool = False
    ) -> None:
        now = self.get_clock().now()
        desired, clamped = self.twist_to_nav_cmd(msg, is_reached=is_reached)

        is_zero = (
            abs(desired.linear_x) <= 1e-9
            and abs(desired.linear_y) <= 1e-9
            and abs(desired.angular_z) <= 1e-9
        )
        if immediate_stop or is_zero or self.last_publish_time is None:
            nav = desired
        else:
            dt = max((now - self.last_publish_time).nanoseconds / 1e9, 1e-3)
            nav = self.apply_accel_limit(desired, dt)
            clamped = clamped or self._nav_differs(nav, desired)

        self.nav_cmd_pub.publish(nav)
        self.last_nav_cmd = nav
        self.last_publish_time = now

        if clamped:
            self.log_clamp(nav)

    def twist_to_nav_cmd(self, msg: Twist, is_reached: int = 0):
        nav = NavCmd()
        raw_x = self._finite_or_zero(msg.linear.x)
        raw_y = self._finite_or_zero(msg.linear.y)
        if self.invert_linear_y:
            raw_y = -raw_y
        raw_z = self.resolve_angular_z(msg, raw_x, raw_y, is_reached)

        nav.linear_x = self._clamp(raw_x, -self.max_linear_x, self.max_linear_x)
        nav.linear_y = self._clamp(raw_y, -self.max_linear_y, self.max_linear_y)
        nav.angular_z = self._clamp(raw_z, -self.max_angular_z, self.max_angular_z)

        speed = math.hypot(nav.linear_x, nav.linear_y)
        if speed > self.max_linear_speed > 0.0:
            scale = self.max_linear_speed / speed
            nav.linear_x *= scale
            nav.linear_y *= scale

        nav.is_reached = int(is_reached)
        return nav, (
            abs(nav.linear_x - raw_x) > 1e-9
            or abs(nav.linear_y - raw_y) > 1e-9
            or abs(nav.angular_z - raw_z) > 1e-9
        )

    def resolve_angular_z(
        self, msg: Twist, linear_x: float, linear_y: float, is_reached: int
    ) -> float:
        if is_reached and self.disable_heading_align_when_reached:
            return 0.0

        if not self.force_zero_angular_z:
            return self._finite_or_zero(msg.angular.z)

        if not self.enable_heading_align:
            return 0.0

        if not self.heading_align_only_when_force_zero_angular_z:
            return 0.0

        speed = math.hypot(linear_x, linear_y)
        if speed < self.heading_align_min_speed:
            return 0.0

        if not self.heading_feedback_warned:
            self.heading_feedback_warned = True
            self.get_logger().warn(
                "Heading align requested, but pb_cmd_vel_to_nav_cmd has no reliable "
                "chassis heading feedback yet; keeping angular_z=0. Set "
                "force_zero_angular_z:=false only after testing angular_z direction."
            )
        return 0.0

    def apply_accel_limit(self, desired: NavCmd, dt: float) -> NavCmd:
        nav = NavCmd()
        max_linear_delta = self.max_linear_accel * dt
        max_angular_delta = self.max_angular_accel * dt
        nav.linear_x = self._limit_delta(
            self.last_nav_cmd.linear_x, desired.linear_x, max_linear_delta
        )
        nav.linear_y = self._limit_delta(
            self.last_nav_cmd.linear_y, desired.linear_y, max_linear_delta
        )
        nav.angular_z = self._limit_delta(
            self.last_nav_cmd.angular_z, desired.angular_z, max_angular_delta
        )
        nav.is_reached = desired.is_reached
        return nav

    def log_clamp(self, nav: NavCmd) -> None:
        now = self.get_clock().now()
        if self.last_clamp_log_time is not None:
            age = (now - self.last_clamp_log_time).nanoseconds / 1e9
            if age < self.log_output_period:
                return
        self.last_clamp_log_time = now
        self.get_logger().warn(
            "NavCmd safety limit active: linear=(%.3f, %.3f), angular_z=%.3f"
            % (nav.linear_x, nav.linear_y, nav.angular_z)
        )

    def _positive_param(self, name: str, fallback: float) -> float:
        value = float(self.get_parameter(name).value)
        if value < 0.0 or not math.isfinite(value):
            return fallback
        return value

    def _bool_param(self, name: str, fallback: bool) -> bool:
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            lowered = value.strip().lower()
            if lowered in ("true", "1", "yes", "on"):
                return True
            if lowered in ("false", "0", "no", "off"):
                return False
        return fallback

    @staticmethod
    def _finite_or_zero(value) -> float:
        value = float(value)
        return value if math.isfinite(value) else 0.0

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    @staticmethod
    def _limit_delta(previous: float, target: float, max_delta: float) -> float:
        return previous + max(-max_delta, min(max_delta, target - previous))

    @staticmethod
    def _nav_differs(a: NavCmd, b: NavCmd) -> bool:
        return (
            abs(a.linear_x - b.linear_x) > 1e-9
            or abs(a.linear_y - b.linear_y) > 1e-9
            or abs(a.angular_z - b.angular_z) > 1e-9
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PbCmdVelToNavCmd()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.nav_cmd_pub.publish(node.twist_to_nav_cmd(Twist(), is_reached=0)[0])
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
