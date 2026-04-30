#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String


class SecondLidarSafetyLimiter(Node):
    def __init__(self) -> None:
        super().__init__("second_lidar_safety_limiter")

        self.declare_parameter("input_cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("output_cmd_vel_topic", "/cmd_vel_safe")
        self.declare_parameter("obstacle_topic", "/second_lidar_obstacle_cloud")
        self.declare_parameter("debug_topic", "/second_lidar_safety_debug")
        self.declare_parameter("frame_id", "gimbal_yaw")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("cmd_vel_timeout_sec", 0.25)
        self.declare_parameter("obstacle_timeout_sec", 0.25)
        self.declare_parameter("emergency_distance", 0.35)
        self.declare_parameter("slow_distance", 0.70)
        self.declare_parameter("caution_distance", 1.20)
        self.declare_parameter("front_angle_deg", 50.0)
        self.declare_parameter("side_angle_deg", 70.0)
        self.declare_parameter("rear_angle_deg", 50.0)
        self.declare_parameter("max_speed_scale_in_caution", 0.6)
        self.declare_parameter("max_speed_scale_in_slow", 0.25)
        self.declare_parameter("emergency_stop", True)
        self.declare_parameter("min_points_for_obstacle", 5)
        self.declare_parameter("min_points_for_emergency", 3)
        self.declare_parameter("enable_front_limit", True)
        self.declare_parameter("enable_back_limit", True)
        self.declare_parameter("enable_left_limit", True)
        self.declare_parameter("enable_right_limit", True)
        self.declare_parameter("pass_through_when_no_cloud", True)

        self.input_cmd_vel_topic = str(self.get_parameter("input_cmd_vel_topic").value)
        self.output_cmd_vel_topic = str(self.get_parameter("output_cmd_vel_topic").value)
        self.obstacle_topic = str(self.get_parameter("obstacle_topic").value)
        self.debug_topic = str(self.get_parameter("debug_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.cmd_vel_timeout_sec = float(self.get_parameter("cmd_vel_timeout_sec").value)
        self.obstacle_timeout_sec = float(self.get_parameter("obstacle_timeout_sec").value)
        self.emergency_distance = float(self.get_parameter("emergency_distance").value)
        self.slow_distance = float(self.get_parameter("slow_distance").value)
        self.caution_distance = float(self.get_parameter("caution_distance").value)
        self.front_angle_rad = math.radians(float(self.get_parameter("front_angle_deg").value))
        self.side_angle_rad = math.radians(float(self.get_parameter("side_angle_deg").value))
        self.rear_angle_rad = math.radians(float(self.get_parameter("rear_angle_deg").value))
        self.max_speed_scale_in_caution = float(
            self.get_parameter("max_speed_scale_in_caution").value
        )
        self.max_speed_scale_in_slow = float(
            self.get_parameter("max_speed_scale_in_slow").value
        )
        self.emergency_stop = self._bool_param("emergency_stop", True)
        self.min_points_for_obstacle = int(self.get_parameter("min_points_for_obstacle").value)
        self.min_points_for_emergency = int(
            self.get_parameter("min_points_for_emergency").value
        )
        self.enable_front_limit = self._bool_param("enable_front_limit", True)
        self.enable_back_limit = self._bool_param("enable_back_limit", True)
        self.enable_left_limit = self._bool_param("enable_left_limit", True)
        self.enable_right_limit = self._bool_param("enable_right_limit", True)
        self.pass_through_when_no_cloud = self._bool_param("pass_through_when_no_cloud", True)

        if self.publish_rate_hz <= 1e-6 or not math.isfinite(self.publish_rate_hz):
            self.publish_rate_hz = 20.0

        self.cmd_sub = self.create_subscription(
            Twist, self.input_cmd_vel_topic, self.on_cmd_vel, 10
        )
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.obstacle_topic,
            self.on_cloud,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self.cmd_pub = self.create_publisher(
            Twist, self.output_cmd_vel_topic, QoSPresetProfiles.SENSOR_DATA.value
        )
        self.debug_pub = self.create_publisher(String, self.debug_topic, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.on_timer)

        self.latest_cmd = Twist()
        self.latest_cmd_time = self.get_clock().now()
        self.has_cmd = False

        self.last_cloud_stamp = None
        self.last_cloud_age = None
        self.last_cloud_frame = ""
        self.stats = self._empty_stats()
        self.frame_mismatch_count = 0

        self.get_logger().info(
            "Second lidar safety limiter ready: %s -> %s, obstacle=%s"
            % (self.input_cmd_vel_topic, self.output_cmd_vel_topic, self.obstacle_topic)
        )

    def on_cmd_vel(self, msg: Twist) -> None:
        self.latest_cmd = msg
        self.latest_cmd_time = self.get_clock().now()
        self.has_cmd = True

    def on_cloud(self, msg: PointCloud2) -> None:
        self.last_cloud_stamp = Time.from_msg(msg.header.stamp)
        self.last_cloud_frame = msg.header.frame_id
        if self.frame_id and msg.header.frame_id and msg.header.frame_id != self.frame_id:
            self.frame_mismatch_count += 1
            if self.frame_mismatch_count % 20 == 1:
                self.get_logger().warn(
                    "Second lidar cloud frame mismatch: expected %s got %s"
                    % (self.frame_id, msg.header.frame_id)
                )

        stats = self._empty_stats()
        for point in point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        ):
            x, y = float(point[0]), float(point[1])
            distance = math.hypot(x, y)
            angle = math.atan2(y, x)

            if self.enable_front_limit and x > 0.0 and abs(angle) <= self.front_angle_rad:
                stats["front_pts"] += 1
                stats["front_min"] = min(stats["front_min"], distance)
            if self.enable_back_limit and x < 0.0 and self._near_back(angle):
                stats["back_pts"] += 1
                stats["back_min"] = min(stats["back_min"], distance)
            if self.enable_left_limit and y > 0.0 and self._near_side(angle):
                stats["left_pts"] += 1
                stats["left_min"] = min(stats["left_min"], distance)
            if self.enable_right_limit and y < 0.0 and self._near_side(angle):
                stats["right_pts"] += 1
                stats["right_min"] = min(stats["right_min"], distance)

        self.stats = stats

    def on_timer(self) -> None:
        now = self.get_clock().now()
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.angular.z = 0.0

        cmd_ok = self.has_cmd and self._age_sec(now, self.latest_cmd_time) <= self.cmd_vel_timeout_sec
        if cmd_ok:
            cmd = self.latest_cmd

        obstacle_ready, obstacle_timeout, cloud_age = self._cloud_status(now)
        self.last_cloud_age = cloud_age

        if not cmd_ok:
            limited_cmd = Twist()
        elif not obstacle_ready:
            limited_cmd = cmd if self.pass_through_when_no_cloud else Twist()
        else:
            limited_cmd = self.apply_limits(cmd)

        self.cmd_pub.publish(limited_cmd)
        self.publish_debug(cmd, limited_cmd, obstacle_timeout)

    def apply_limits(self, cmd: Twist) -> Twist:
        scale_vx_pos, scale_vx_neg, scale_vy_pos, scale_vy_neg, emergency = self.compute_scales()

        out = Twist()
        out.angular.z = cmd.angular.z

        if emergency and self.emergency_stop:
            out.linear.x = 0.0
            out.linear.y = 0.0
            return out

        if cmd.linear.x > 0.0:
            out.linear.x = cmd.linear.x * scale_vx_pos
        elif cmd.linear.x < 0.0:
            out.linear.x = cmd.linear.x * scale_vx_neg
        else:
            out.linear.x = 0.0

        if cmd.linear.y > 0.0:
            out.linear.y = cmd.linear.y * scale_vy_pos
        elif cmd.linear.y < 0.0:
            out.linear.y = cmd.linear.y * scale_vy_neg
        else:
            out.linear.y = 0.0

        return out

    def compute_scales(self):
        scale_vx_pos = 1.0
        scale_vx_neg = 1.0
        scale_vy_pos = 1.0
        scale_vy_neg = 1.0
        emergency = False

        scale_vx_pos, emergency = self._scale_for_region(
            "front", scale_vx_pos, emergency
        )
        scale_vx_neg, emergency = self._scale_for_region("back", scale_vx_neg, emergency)
        scale_vy_pos, emergency = self._scale_for_region("left", scale_vy_pos, emergency)
        scale_vy_neg, emergency = self._scale_for_region("right", scale_vy_neg, emergency)

        return (
            max(0.0, min(scale_vx_pos, 1.0)),
            max(0.0, min(scale_vx_neg, 1.0)),
            max(0.0, min(scale_vy_pos, 1.0)),
            max(0.0, min(scale_vy_neg, 1.0)),
            emergency,
        )

    def _scale_for_region(self, name: str, scale: float, emergency: bool):
        min_dist = self.stats[f"{name}_min"]
        count = self.stats[f"{name}_pts"]

        if count >= self.min_points_for_emergency and min_dist <= self.emergency_distance:
            emergency = True
            return 0.0, emergency

        if count >= self.min_points_for_obstacle:
            if min_dist <= self.slow_distance:
                scale = min(scale, self.max_speed_scale_in_slow)
            elif min_dist <= self.caution_distance:
                scale = min(scale, self.max_speed_scale_in_caution)

        return scale, emergency

    def publish_debug(self, cmd_in: Twist, cmd_out: Twist, obstacle_timeout: bool) -> None:
        cloud_age = self.last_cloud_age
        cloud_age_text = "inf" if cloud_age is None or not math.isfinite(cloud_age) else f"{cloud_age:.2f}"

        msg = String()
        msg.data = (
            "second_lidar_safety: "
            f"cloud_age={cloud_age_text} obstacle_timeout={str(obstacle_timeout).lower()} "
            f"front_min={self._fmt_dist(self.stats['front_min'])} front_pts={self.stats['front_pts']} "
            f"back_min={self._fmt_dist(self.stats['back_min'])} back_pts={self.stats['back_pts']} "
            f"left_min={self._fmt_dist(self.stats['left_min'])} left_pts={self.stats['left_pts']} "
            f"right_min={self._fmt_dist(self.stats['right_min'])} right_pts={self.stats['right_pts']} "
            f"scale_vx_pos={self._fmt_scale(self._scale_value('front'))} "
            f"scale_vx_neg={self._fmt_scale(self._scale_value('back'))} "
            f"scale_vy_pos={self._fmt_scale(self._scale_value('left'))} "
            f"scale_vy_neg={self._fmt_scale(self._scale_value('right'))} "
            f"cmd_in=({cmd_in.linear.x:.2f},{cmd_in.linear.y:.2f},{cmd_in.angular.z:.2f}) "
            f"cmd_out=({cmd_out.linear.x:.2f},{cmd_out.linear.y:.2f},{cmd_out.angular.z:.2f})"
        )
        self.debug_pub.publish(msg)

    def _scale_value(self, name: str) -> float:
        scale, _ = self._scale_for_region(name, 1.0, False)
        return scale

    def _cloud_status(self, now: Time):
        if self.last_cloud_stamp is None:
            return False, True, math.inf
        cloud_age = self._age_sec(now, self.last_cloud_stamp)
        obstacle_timeout = cloud_age > self.obstacle_timeout_sec
        return not obstacle_timeout, obstacle_timeout, cloud_age

    @staticmethod
    def _age_sec(now: Time, stamp: Time) -> float:
        return max(0.0, (now - stamp).nanoseconds / 1e9)

    def _near_back(self, angle: float) -> bool:
        return abs(math.pi - abs(angle)) <= self.rear_angle_rad

    def _near_side(self, angle: float) -> bool:
        return abs(abs(angle) - math.pi / 2.0) <= self.side_angle_rad

    @staticmethod
    def _fmt_dist(value: float) -> str:
        return "inf" if not math.isfinite(value) else f"{value:.2f}"

    @staticmethod
    def _fmt_scale(value: float) -> str:
        return f"{value:.2f}"

    @staticmethod
    def _empty_stats():
        return {
            "front_min": math.inf,
            "back_min": math.inf,
            "left_min": math.inf,
            "right_min": math.inf,
            "front_pts": 0,
            "back_pts": 0,
            "left_pts": 0,
            "right_pts": 0,
        }

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


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SecondLidarSafetyLimiter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
