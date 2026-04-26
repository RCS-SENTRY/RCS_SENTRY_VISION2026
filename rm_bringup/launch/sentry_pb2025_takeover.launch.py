"""
XMU sentry bringup with PB2025 taking over navigation.

XMU keeps communication, camera, vision, autoaim, and optional decision nodes.
PB2025 owns the complete navigation pipeline. The old XMU navigation launch files
are intentionally not included here.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _truthy(name):
    return PythonExpression(["'", LaunchConfiguration(name), "'.lower() in ['true', '1', 'yes']"])


def _falsey(name):
    return PythonExpression(["'", LaunchConfiguration(name), "'.lower() not in ['true', '1', 'yes']"])


def _and(*parts):
    expression = []
    for index, part in enumerate(parts):
        if index:
            expression.append(" and ")
        expression.append(part)
    return PythonExpression(expression)


def _or(*parts):
    expression = []
    for index, part in enumerate(parts):
        if index:
            expression.append(" or ")
        expression.append(part)
    return PythonExpression(expression)


def generate_launch_description():
    hw_bridge_dir = get_package_share_directory("rm_hw_bridge")
    hik_driver_dir = get_package_share_directory("rm_hik_driver")
    vision_dir = get_package_share_directory("rm_vision")
    autoaim_dir = get_package_share_directory("rm_autoaim")
    bringup_dir = get_package_share_directory("rm_bringup")

    hw_bridge_params = os.path.join(hw_bridge_dir, "config", "params.yaml")
    hik_params = os.path.join(hik_driver_dir, "config", "params.yaml")
    vision_params = os.path.join(vision_dir, "config", "params.yaml")
    autoaim_params = os.path.join(autoaim_dir, "config", "params.yaml")
    pb_nav_launch = os.path.join(bringup_dir, "launch", "pb2025_takeover_bringup.launch.py")

    full_xmu_stack_enabled = _and(_falsey("navigation_only"), _truthy("enable_xmu_vision_stack"))
    serial_hw_enabled = _truthy("use_serial")
    delayed_vision_enabled = _and(_truthy("use_serial"), full_xmu_stack_enabled)
    immediate_vision_enabled = _and(_falsey("use_serial"), full_xmu_stack_enabled)
    decision_enabled = _and(_truthy("enable_decision"), _falsey("navigation_only"))
    pb_navigation_enabled = _or(_truthy("enable_pb_navigation"), _truthy("navigation_only"))

    return LaunchDescription([
        DeclareLaunchArgument("use_serial", default_value="true"),
        DeclareLaunchArgument("serial_device", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baudrate", default_value="460800"),
        DeclareLaunchArgument("navigation_only", default_value="false"),
        DeclareLaunchArgument("enable_pb_navigation", default_value="true"),
        DeclareLaunchArgument("enable_xmu_vision_stack", default_value="true"),
        DeclareLaunchArgument("enable_decision", default_value="false"),
        DeclareLaunchArgument("color_ignore", default_value="1"),
        DeclareLaunchArgument("target_color", default_value="red"),
        DeclareLaunchArgument("publish_debug_image", default_value="false"),
        DeclareLaunchArgument("model_path", default_value="model/yolo11.xml"),

        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("slam", default_value="False"),
        DeclareLaunchArgument("map", default_value=""),
        DeclareLaunchArgument("prior_pcd_file", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(bringup_dir, "config", "pb2025_xmu_nav_params.yaml"),
        ),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("use_respawn", default_value="False"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("enable_small_gicp", default_value="false"),
        DeclareLaunchArgument("enable_prior_pcd", default_value="false"),
        DeclareLaunchArgument("cmd_vel_input_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("nav_cmd_output_topic", default_value="/nav_cmd"),
        DeclareLaunchArgument("force_zero_angular_z", default_value="true"),
        DeclareLaunchArgument("invert_linear_y", default_value="false"),
        DeclareLaunchArgument("publish_livox_frame_alias", default_value="false"),
        DeclareLaunchArgument("lidar_x", default_value="0.0"),
        DeclareLaunchArgument("lidar_y", default_value="0.2"),
        DeclareLaunchArgument("lidar_z", default_value="0.35"),
        DeclareLaunchArgument("lidar_roll", default_value="0.0"),
        DeclareLaunchArgument("lidar_pitch", default_value="0.3115"),
        DeclareLaunchArgument("lidar_yaw", default_value="1.5708"),

        Node(
            package="rm_hw_bridge",
            executable="hw_bridge_node",
            name="rm_hw_bridge",
            output="screen",
            parameters=[
                hw_bridge_params,
                {
                    "serial_device": LaunchConfiguration("serial_device"),
                    "baudrate": LaunchConfiguration("baudrate"),
                },
            ],
            condition=IfCondition(serial_hw_enabled),
        ),

        TimerAction(
            period=2.0,
            condition=IfCondition(delayed_vision_enabled),
            actions=[
                Node(
                    package="rm_hik_driver",
                    executable="hik_camera_node",
                    name="rm_hik_driver",
                    output="screen",
                    parameters=[hik_params],
                )
            ],
        ),
        Node(
            package="rm_hik_driver",
            executable="hik_camera_node",
            name="rm_hik_driver",
            output="screen",
            parameters=[hik_params],
            condition=IfCondition(immediate_vision_enabled),
        ),
        TimerAction(
            period=4.0,
            condition=IfCondition(delayed_vision_enabled),
            actions=[
                Node(
                    package="rm_vision",
                    executable="vision_detector_node",
                    name="rm_vision_detector",
                    output="screen",
                    parameters=[
                        vision_params,
                        {
                            "color_ignore": LaunchConfiguration("color_ignore"),
                            "target_color": LaunchConfiguration("target_color"),
                            "publish_debug_image": LaunchConfiguration("publish_debug_image"),
                            "model_path": LaunchConfiguration("model_path"),
                        },
                    ],
                ),
                Node(
                    package="rm_autoaim",
                    executable="autoaim_node",
                    name="rm_autoaim",
                    output="screen",
                    parameters=[autoaim_params],
                ),
            ],
        ),
        TimerAction(
            period=2.0,
            condition=IfCondition(immediate_vision_enabled),
            actions=[
                Node(
                    package="rm_vision",
                    executable="vision_detector_node",
                    name="rm_vision_detector",
                    output="screen",
                    parameters=[
                        vision_params,
                        {
                            "color_ignore": LaunchConfiguration("color_ignore"),
                            "target_color": LaunchConfiguration("target_color"),
                            "publish_debug_image": LaunchConfiguration("publish_debug_image"),
                            "model_path": LaunchConfiguration("model_path"),
                        },
                    ],
                ),
                Node(
                    package="rm_autoaim",
                    executable="autoaim_node",
                    name="rm_autoaim",
                    output="screen",
                    parameters=[autoaim_params],
                ),
            ],
        ),
        TimerAction(
            period=5.0,
            condition=IfCondition(decision_enabled),
            actions=[
                Node(
                    package="sentry_bt",
                    executable="sentry_bt",
                    name="sentry_bt",
                    output="screen",
                )
            ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pb_nav_launch),
            condition=IfCondition(pb_navigation_enabled),
            launch_arguments={
                "namespace": LaunchConfiguration("namespace"),
                "slam": LaunchConfiguration("slam"),
                "map": LaunchConfiguration("map"),
                "prior_pcd_file": LaunchConfiguration("prior_pcd_file"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "params_file": LaunchConfiguration("params_file"),
                "autostart": LaunchConfiguration("autostart"),
                "use_respawn": LaunchConfiguration("use_respawn"),
                "log_level": LaunchConfiguration("log_level"),
                "rviz": LaunchConfiguration("rviz"),
                "enable_small_gicp": LaunchConfiguration("enable_small_gicp"),
                "enable_prior_pcd": LaunchConfiguration("enable_prior_pcd"),
                "enable_cmd_bridge": "true",
                "cmd_vel_input_topic": LaunchConfiguration("cmd_vel_input_topic"),
                "nav_cmd_output_topic": LaunchConfiguration("nav_cmd_output_topic"),
                "force_zero_angular_z": LaunchConfiguration("force_zero_angular_z"),
                "invert_linear_y": LaunchConfiguration("invert_linear_y"),
                "publish_livox_frame_alias": LaunchConfiguration("publish_livox_frame_alias"),
                "lidar_x": LaunchConfiguration("lidar_x"),
                "lidar_y": LaunchConfiguration("lidar_y"),
                "lidar_z": LaunchConfiguration("lidar_z"),
                "lidar_roll": LaunchConfiguration("lidar_roll"),
                "lidar_pitch": LaunchConfiguration("lidar_pitch"),
                "lidar_yaw": LaunchConfiguration("lidar_yaw"),
            }.items(),
        ),
    ])
