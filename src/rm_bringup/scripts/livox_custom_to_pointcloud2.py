#!/usr/bin/env python3

from typing import List, Tuple

import rclpy
from livox_ros_driver2.msg import CustomMsg
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class LivoxCustomToPointCloud2(Node):
    def __init__(self) -> None:
        super().__init__("livox_custom_to_pointcloud2")

        self.declare_parameter("input_topic", "/livox/lidar_192_168_1_166")
        self.declare_parameter("output_topic", "/second_livox/lidar")
        self.declare_parameter("output_frame", "second_mid360")
        self.declare_parameter("log_period_sec", 2.0)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.output_frame = str(self.get_parameter("output_frame").value)
        self.log_period = max(float(self.get_parameter("log_period_sec").value), 0.5)

        self.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        self.pub = self.create_publisher(
            PointCloud2, self.output_topic, QoSPresetProfiles.SENSOR_DATA.value
        )
        self.sub = self.create_subscription(
            CustomMsg,
            self.input_topic,
            self.on_cloud,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self.log_timer = self.create_timer(self.log_period, self.log_stats)

        self.received_count = 0
        self.published_count = 0
        self.last_point_count = 0
        self.last_receive_time = None

        self.get_logger().info(
            "Livox CustomMsg converter ready: %s -> %s, frame=%s"
            % (self.input_topic, self.output_topic, self.output_frame)
        )

    def on_cloud(self, msg: CustomMsg) -> None:
        self.received_count += 1
        self.last_receive_time = self.get_clock().now()

        points: List[Tuple[float, float, float, float]] = [
            (float(point.x), float(point.y), float(point.z), float(point.reflectivity))
            for point in msg.points
        ]
        self.last_point_count = len(points)

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.output_frame
        self.pub.publish(point_cloud2.create_cloud(header, self.fields, points))
        self.published_count += 1

    def log_stats(self) -> None:
        if self.last_receive_time is None:
            age_text = "never"
        else:
            age = self.get_clock().now() - self.last_receive_time
            age_text = "%.3fs" % (age.nanoseconds / 1e9)
        self.get_logger().debug(
            "converted=%d received=%d last_points=%d last_age=%s"
            % (self.published_count, self.received_count, self.last_point_count, age_text)
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LivoxCustomToPointCloud2()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as exc:
        if "Unable to convert call argument to Python object" not in str(exc):
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
