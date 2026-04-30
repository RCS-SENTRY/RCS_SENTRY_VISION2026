"""XMU sentry top-level bringup.

This is the only whole-robot entry point. XMU keeps communication, camera,
vision, and autoaim. PB2025 owns navigation, and the PB cmd_vel output is
bridged to the existing XMU NavCmd serial protocol.
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
    pb_nav_launch = os.path.join(bringup_dir, "launch", "sentry_pb2025_takeover.launch.py")

    debug_no_serial = _truthy("debug_no_serial")
    serial_enabled = _and(_truthy("use_serial"), _falsey("debug_no_serial"))
    vision_stack_enabled = _and(_falsey("navigation_only"), _truthy("enable_vision"))
    delayed_vision_enabled = _and(serial_enabled, vision_stack_enabled)
    immediate_vision_enabled = _and(_or(_falsey("use_serial"), debug_no_serial), vision_stack_enabled)
    navigation_enabled = _and(_or(_truthy("enable_navigation"), _truthy("navigation_only")), _falsey("vision_only"))
    decision_enabled = _and(_truthy("enable_decision"), _falsey("navigation_only"), _falsey("vision_only"))

    return LaunchDescription([
        DeclareLaunchArgument("use_serial", default_value="true"),
        DeclareLaunchArgument("serial_device", default_value="/dev/rm_serial"),
        DeclareLaunchArgument("baudrate", default_value="460800"),
        DeclareLaunchArgument("navigation_only", default_value="false"),
        DeclareLaunchArgument("vision_only", default_value="false"),
        DeclareLaunchArgument("debug_no_serial", default_value="false"),
        DeclareLaunchArgument("enable_navigation", default_value="true"),
        DeclareLaunchArgument("enable_vision", default_value="true"),
        DeclareLaunchArgument("enable_decision", default_value="false"),
        DeclareLaunchArgument("color_ignore", default_value="1"),
        DeclareLaunchArgument("target_color", default_value="red"),
        DeclareLaunchArgument("publish_debug_image", default_value="false"),
        DeclareLaunchArgument("model_path", default_value="model/yolo11.xml"),

        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("slam", default_value="False"),
        DeclareLaunchArgument("map", default_value="/home/rm/Desktop/SENTRY_FULL/maps/self_filtered_map.yaml"),
        DeclareLaunchArgument("prior_pcd_file", default_value="/home/rm/Desktop/SENTRY_FULL/maps/self_filtered_scans.pcd"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(bringup_dir, "config", "pb2025_xmu_nav_params.yaml"),
        ),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("use_respawn", default_value="False"),
        DeclareLaunchArgument("log_level", default_value="info"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("enable_small_gicp", default_value="false"),
        DeclareLaunchArgument("enable_prior_pcd", default_value="false"),
        DeclareLaunchArgument("cmd_vel_input_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("cmd_vel_safe_topic", default_value="/cmd_vel_safe"),
        DeclareLaunchArgument("nav_cmd_output_topic", default_value="/nav_cmd"),
        DeclareLaunchArgument("publish_rate_hz", default_value="20.0"),
        DeclareLaunchArgument("cmd_vel_timeout_sec", default_value="0.25"),
        DeclareLaunchArgument("goal_reached_latch_sec", default_value="1.0"),
        DeclareLaunchArgument("force_zero_angular_z", default_value="true"),
        DeclareLaunchArgument("invert_linear_y", default_value="false"),
        DeclareLaunchArgument("publish_livox_frame_alias", default_value="false"),
        DeclareLaunchArgument("lidar_x", default_value="0.0"),
        DeclareLaunchArgument("lidar_y", default_value="0.2"),
        DeclareLaunchArgument("lidar_z", default_value="0.35"),
        DeclareLaunchArgument("lidar_roll", default_value="0.0"),
        DeclareLaunchArgument("lidar_pitch", default_value="0.3115"),
        DeclareLaunchArgument("lidar_yaw", default_value="1.5708"),
        DeclareLaunchArgument("enable_second_lidar_safety", default_value="false"),
        DeclareLaunchArgument("second_lidar_frame", default_value="second_mid360"),
        DeclareLaunchArgument("second_lidar_x", default_value="0.0"),
        DeclareLaunchArgument("second_lidar_y", default_value="-0.2"),
        DeclareLaunchArgument("second_lidar_z", default_value="0.35"),
        DeclareLaunchArgument("second_lidar_roll", default_value="0.0"),
        DeclareLaunchArgument("second_lidar_pitch", default_value="0.3115"),
        DeclareLaunchArgument("second_lidar_yaw", default_value="-1.5708"),
        DeclareLaunchArgument("second_lidar_filter_target_frame", default_value="gimbal_yaw"),
        DeclareLaunchArgument(
            "dual_lidar_user_config_path",
            default_value=os.path.join(bringup_dir, "config", "pb2025_xmu_dual_mid360_config.json"),
        ),
        DeclareLaunchArgument("front_lidar_custom_topic", default_value="livox/lidar_192_168_1_173"),
        DeclareLaunchArgument("front_imu_custom_topic", default_value="livox/imu_192_168_1_173"),
        DeclareLaunchArgument("second_lidar_custom_topic", default_value="/livox/lidar_192_168_1_166"),
        DeclareLaunchArgument("second_lidar_pointcloud_topic", default_value="/second_livox/lidar"),

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
            condition=IfCondition(serial_enabled),
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
            condition=IfCondition(navigation_enabled),
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
                "use_rviz": LaunchConfiguration("use_rviz"),
                "enable_small_gicp": LaunchConfiguration("enable_small_gicp"),
                "enable_prior_pcd": LaunchConfiguration("enable_prior_pcd"),
                "enable_cmd_bridge": "true",
                "cmd_vel_input_topic": LaunchConfiguration("cmd_vel_input_topic"),
                "cmd_vel_safe_topic": LaunchConfiguration("cmd_vel_safe_topic"),
                "nav_cmd_output_topic": LaunchConfiguration("nav_cmd_output_topic"),
                "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
                "cmd_vel_timeout_sec": LaunchConfiguration("cmd_vel_timeout_sec"),
                "goal_reached_latch_sec": LaunchConfiguration("goal_reached_latch_sec"),
                "force_zero_angular_z": LaunchConfiguration("force_zero_angular_z"),
                "invert_linear_y": LaunchConfiguration("invert_linear_y"),
                "publish_livox_frame_alias": LaunchConfiguration("publish_livox_frame_alias"),
                "lidar_x": LaunchConfiguration("lidar_x"),
                "lidar_y": LaunchConfiguration("lidar_y"),
                "lidar_z": LaunchConfiguration("lidar_z"),
                "lidar_roll": LaunchConfiguration("lidar_roll"),
                "lidar_pitch": LaunchConfiguration("lidar_pitch"),
                "lidar_yaw": LaunchConfiguration("lidar_yaw"),
                "enable_second_lidar_safety": LaunchConfiguration("enable_second_lidar_safety"),
                "second_lidar_frame": LaunchConfiguration("second_lidar_frame"),
                "second_lidar_x": LaunchConfiguration("second_lidar_x"),
                "second_lidar_y": LaunchConfiguration("second_lidar_y"),
                "second_lidar_z": LaunchConfiguration("second_lidar_z"),
                "second_lidar_roll": LaunchConfiguration("second_lidar_roll"),
                "second_lidar_pitch": LaunchConfiguration("second_lidar_pitch"),
                "second_lidar_yaw": LaunchConfiguration("second_lidar_yaw"),
                "second_lidar_filter_target_frame": LaunchConfiguration("second_lidar_filter_target_frame"),
                "dual_lidar_user_config_path": LaunchConfiguration("dual_lidar_user_config_path"),
                "front_lidar_custom_topic": LaunchConfiguration("front_lidar_custom_topic"),
                "front_imu_custom_topic": LaunchConfiguration("front_imu_custom_topic"),
                "second_lidar_custom_topic": LaunchConfiguration("second_lidar_custom_topic"),
                "second_lidar_pointcloud_topic": LaunchConfiguration("second_lidar_pointcloud_topic"),
            }.items(),
        ),
    ])
