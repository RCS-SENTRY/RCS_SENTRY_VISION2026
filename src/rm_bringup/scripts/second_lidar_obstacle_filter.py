#!/usr/bin/env python3

import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener


class SecondLidarObstacleFilter(Node):
    def __init__(self) -> None:
        super().__init__("second_lidar_obstacle_filter")

        self.declare_parameter("input_topic", "/second_livox/lidar")
        self.declare_parameter("output_topic", "/second_lidar_obstacle_cloud")
        self.declare_parameter("debug_topic", "/second_lidar_obstacle_debug")
        self.declare_parameter("target_frame", "gimbal_yaw")
        self.declare_parameter("source_frame", "second_mid360")
        self.declare_parameter("min_range", 0.20)
        self.declare_parameter("max_range", 3.00)
        self.declare_parameter("min_height", 0.15)
        self.declare_parameter("max_height", 1.10)
        self.declare_parameter("voxel_leaf_size", 0.10)
        self.declare_parameter("body_box_x_min", -0.30)
        self.declare_parameter("body_box_x_max", 0.30)
        self.declare_parameter("body_box_y_min", -0.30)
        self.declare_parameter("body_box_y_max", 0.30)
        self.declare_parameter("body_box_z_min", -0.10)
        self.declare_parameter("body_box_z_max", 0.75)
        self.declare_parameter("allow_latest_tf_fallback", False)
        self.declare_parameter("debug_log_period_sec", 1.0)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.debug_topic = str(self.get_parameter("debug_topic").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.source_frame = str(self.get_parameter("source_frame").value)
        self.min_range = float(self.get_parameter("min_range").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.min_height = float(self.get_parameter("min_height").value)
        self.max_height = float(self.get_parameter("max_height").value)
        self.voxel_leaf_size = float(self.get_parameter("voxel_leaf_size").value)
        self.body_box_x_min = float(self.get_parameter("body_box_x_min").value)
        self.body_box_x_max = float(self.get_parameter("body_box_x_max").value)
        self.body_box_y_min = float(self.get_parameter("body_box_y_min").value)
        self.body_box_y_max = float(self.get_parameter("body_box_y_max").value)
        self.body_box_z_min = float(self.get_parameter("body_box_z_min").value)
        self.body_box_z_max = float(self.get_parameter("body_box_z_max").value)
        self.allow_latest_tf_fallback = self._bool_param("allow_latest_tf_fallback", False)
        debug_period = max(float(self.get_parameter("debug_log_period_sec").value), 0.2)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cloud_pub = self.create_publisher(
            PointCloud2, self.output_topic, QoSPresetProfiles.SENSOR_DATA.value
        )
        self.debug_pub = self.create_publisher(DiagnosticArray, self.debug_topic, 10)
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.on_cloud,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self.debug_timer = self.create_timer(debug_period, self.publish_debug)

        self.received_cloud_count = 0
        self.published_cloud_count = 0
        self.dropped_tf_count = 0
        self.empty_after_filter_count = 0
        self.point_count_in = 0
        self.point_count_out = 0
        self.last_source_frame = self.source_frame

        if self.allow_latest_tf_fallback:
            self.get_logger().warn(
                "allow_latest_tf_fallback is ignored; stamped TF is required for obstacle cloud"
            )

        self.get_logger().info(
            "Second lidar obstacle filter ready: %s -> %s, target_frame=%s"
            % (self.input_topic, self.output_topic, self.target_frame)
        )

    def on_cloud(self, msg: PointCloud2) -> None:
        self.received_cloud_count += 1
        source_frame = self.source_frame or msg.header.frame_id
        stamp = Time.from_msg(msg.header.stamp)
        self.last_source_frame = source_frame

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                stamp,
                timeout=Duration(seconds=0.05),
            )
        except TransformException as exc:
            self.dropped_tf_count += 1
            if self.dropped_tf_count % 20 == 1:
                self.get_logger().warn("Dropping second lidar cloud: %s" % str(exc))
            return

        rotation, translation = self.transform_to_matrix(transform)
        voxel_points = {}
        point_count_in = 0

        for point in point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        ):
            point_count_in += 1
            x, y, z = self.transform_point(point, rotation, translation)

            horizontal_range = math.hypot(x, y)
            if horizontal_range < self.min_range or horizontal_range > self.max_range:
                continue
            if z < self.min_height or z > self.max_height:
                continue
            if (
                self.body_box_x_min <= x <= self.body_box_x_max
                and self.body_box_y_min <= y <= self.body_box_y_max
                and self.body_box_z_min <= z <= self.body_box_z_max
            ):
                continue

            if self.voxel_leaf_size > 1e-6:
                key = (
                    math.floor(x / self.voxel_leaf_size),
                    math.floor(y / self.voxel_leaf_size),
                    math.floor(z / self.voxel_leaf_size),
                )
                voxel_points.setdefault(key, (x, y, z))
            else:
                voxel_points[(point_count_in, 0, 0)] = (x, y, z)

        filtered_points = list(voxel_points.values())
        self.point_count_in = point_count_in
        self.point_count_out = len(filtered_points)
        if not filtered_points:
            self.empty_after_filter_count += 1

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.target_frame
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, filtered_points))
        self.published_cloud_count += 1

    def publish_debug(self) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = "second_lidar_obstacle_filter"
        status.hardware_id = self.source_frame
        status.level = DiagnosticStatus.OK
        status.message = "ok"
        status.values = [
            self.kv("received_cloud_count", self.received_cloud_count),
            self.kv("published_cloud_count", self.published_cloud_count),
            self.kv("dropped_tf_count", self.dropped_tf_count),
            self.kv("empty_after_filter_count", self.empty_after_filter_count),
            self.kv("point_count_in", self.point_count_in),
            self.kv("point_count_out", self.point_count_out),
            self.kv("target_frame", self.target_frame),
            self.kv("source_frame", self.last_source_frame),
        ]
        array.status.append(status)
        self.debug_pub.publish(array)

    @staticmethod
    def transform_to_matrix(transform):
        q = transform.transform.rotation
        t = transform.transform.translation
        x, y, z, w = q.x, q.y, q.z, q.w
        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz = x * y, x * z, y * z
        wx, wy, wz = w * x, w * y, w * z
        rotation = (
            (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
            (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
            (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
        )
        translation = (t.x, t.y, t.z)
        return rotation, translation

    @staticmethod
    def transform_point(point, rotation, translation):
        x, y, z = float(point[0]), float(point[1]), float(point[2])
        return (
            rotation[0][0] * x + rotation[0][1] * y + rotation[0][2] * z + translation[0],
            rotation[1][0] * x + rotation[1][1] * y + rotation[1][2] * z + translation[1],
            rotation[2][0] * x + rotation[2][1] * y + rotation[2][2] * z + translation[2],
        )

    @staticmethod
    def kv(key: str, value) -> KeyValue:
        item = KeyValue()
        item.key = key
        item.value = str(value)
        return item

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
    node = SecondLidarObstacleFilter()
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
