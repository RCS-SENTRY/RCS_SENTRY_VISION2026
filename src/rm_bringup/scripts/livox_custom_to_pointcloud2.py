#!/usr/bin/env python3

import rclpy
from livox_ros_driver2.msg import CustomMsg
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class LivoxCustomToPointCloud2(Node):
    def __init__(self) -> None:
        super().__init__("livox_custom_to_pointcloud2")
        self.declare_parameter("input_topic", "/livox/lidar")
        self.declare_parameter("output_topic", "/livox/lidar/pointcloud")
        self.declare_parameter("frame_id", "")

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        self.publisher = self.create_publisher(
            PointCloud2, self.output_topic, QoSPresetProfiles.SENSOR_DATA.value
        )
        self.subscription = self.create_subscription(
            CustomMsg, self.input_topic, self.on_msg, QoSPresetProfiles.SENSOR_DATA.value
        )
        self.get_logger().info(
            "Livox CustomMsg converter ready: %s -> %s" % (self.input_topic, self.output_topic)
        )

    def on_msg(self, msg: CustomMsg) -> None:
        header = msg.header
        if self.frame_id:
            header.frame_id = self.frame_id
        points = ((point.x, point.y, point.z) for point in msg.points)
        self.publisher.publish(point_cloud2.create_cloud_xyz32(header, points))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LivoxCustomToPointCloud2()
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
