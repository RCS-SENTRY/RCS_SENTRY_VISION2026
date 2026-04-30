"""Standalone second MID-360 driver entry.

This launch is intentionally independent from the PB2025 navigation launch.
It publishes the second lidar on /second_livox/* and must not be included by
default in the main navigation bringup.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _validate_config(context, *args, **kwargs):
    user_config_path = LaunchConfiguration("user_config_path").perform(context)
    if not os.path.exists(user_config_path):
        raise RuntimeError(f"Second MID-360 config not found: {user_config_path}")
    return []


def generate_launch_description():
    bringup_dir = get_package_share_directory("rm_bringup")
    default_config = os.path.join(
        bringup_dir,
        "config",
        "pb2025_xmu_second_mid360_config.json",
    )

    return LaunchDescription([
        DeclareLaunchArgument("user_config_path", default_value=default_config),
        DeclareLaunchArgument("frame_id", default_value="second_mid360"),
        DeclareLaunchArgument("lidar_topic", default_value="/second_livox/lidar"),
        DeclareLaunchArgument("imu_topic", default_value="/second_livox/imu"),
        DeclareLaunchArgument("pointcloud_topic", default_value="/second_livox/lidar/pointcloud"),
        DeclareLaunchArgument("publish_freq", default_value="10.0"),
        DeclareLaunchArgument("xfer_format", default_value="0"),
        DeclareLaunchArgument("multi_topic", default_value="0"),
        DeclareLaunchArgument("output_data_type", default_value="0"),
        DeclareLaunchArgument("log_level", default_value="info"),

        OpaqueFunction(function=_validate_config),

        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="second_livox_driver",
            output="screen",
            parameters=[{
                "xfer_format": ParameterValue(LaunchConfiguration("xfer_format"), value_type=int),
                "multi_topic": ParameterValue(LaunchConfiguration("multi_topic"), value_type=int),
                "data_src": 0,
                "publish_freq": ParameterValue(LaunchConfiguration("publish_freq"), value_type=float),
                "output_data_type": ParameterValue(LaunchConfiguration("output_data_type"), value_type=int),
                "frame_id": LaunchConfiguration("frame_id"),
                "lvx_file_path": "",
                "user_config_path": LaunchConfiguration("user_config_path"),
                "cmdline_input_bd_code": "livox0000000001",
            }],
            remappings=[
                ("livox/lidar", LaunchConfiguration("lidar_topic")),
                ("livox/imu", LaunchConfiguration("imu_topic")),
                ("livox/lidar/pointcloud", LaunchConfiguration("pointcloud_topic")),
            ],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        ),
    ])
