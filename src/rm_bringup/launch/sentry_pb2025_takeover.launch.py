"""PB2025 navigation takeover for the XMU sentry.

This launch starts the PB2025 navigation chain only. It does not include any
old XMU navigation nodes, fallbacks, or recovery managers.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def _is_true(name):
    return PythonExpression(["'", LaunchConfiguration(name), "'.lower() in ['true', '1', 'yes']"])


def _is_false(name):
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


def _validate_paths(context, *args, **kwargs):
    slam = LaunchConfiguration("slam").perform(context).lower() in ("true", "1", "yes")
    map_yaml = LaunchConfiguration("map").perform(context)
    params_file = LaunchConfiguration("params_file").perform(context)

    if not os.path.exists(params_file):
        raise RuntimeError(f"PB2025 XMU params file not found: {params_file}")
    if not slam:
        if not map_yaml:
            raise RuntimeError("slam:=False requires map:=/absolute/path/to/map.yaml")
        if not os.path.exists(map_yaml):
            raise RuntimeError(f"Nav2 map yaml not found: {map_yaml}")
    return []


def _navigation_env():
    library_paths = [
        path
        for path in os.environ.get("LD_LIBRARY_PATH", "").split(":")
        if path and not path.startswith("/opt/MVS")
    ]
    return {
        "LD_LIBRARY_PATH": ":".join(library_paths),
    }


def generate_launch_description():
    bringup_dir = get_package_share_directory("rm_bringup")
    navigation_env = _navigation_env()

    namespace = LaunchConfiguration("namespace")
    slam = LaunchConfiguration("slam")
    map_yaml_file = LaunchConfiguration("map")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={
                "use_sim_time": use_sim_time,
                "yaml_filename": map_yaml_file,
                "local_costmap.local_costmap.ros__parameters.second_lidar_obstacle_layer.enabled": LaunchConfiguration("enable_second_lidar_costmap"),
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    no_slam = _is_false("slam")
    use_slam = _is_true("slam")
    small_gicp_enabled = _and(_is_false("slam"), _is_true("enable_small_gicp"))
    static_map_to_odom_enabled = _or(
        _is_true("slam"),
        _and(_is_false("slam"), _is_false("enable_small_gicp")),
    )
    cmd_bridge_enabled = _is_true("enable_cmd_bridge")
    rviz_enabled = _is_true("use_rviz")
    second_lidar_enabled = _is_true("enable_second_lidar")
    second_lidar_filter_enabled = _and(
        second_lidar_enabled, _is_true("enable_second_lidar_filter")
    )
    second_lidar_safety_enabled = _and(
        second_lidar_enabled, _is_true("enable_second_lidar_safety_limiter")
    )
    cmd_bridge_safe_enabled = _and(cmd_bridge_enabled, second_lidar_safety_enabled)
    cmd_bridge_direct_enabled = _and(
        cmd_bridge_enabled,
        _or(_is_false("enable_second_lidar"), _is_false("enable_second_lidar_safety_limiter")),
    )

    return LaunchDescription([
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),

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
        DeclareLaunchArgument("enable_cmd_bridge", default_value="true"),
        DeclareLaunchArgument("cmd_vel_input_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("cmd_vel_safe_topic", default_value="/cmd_vel_safe"),
        DeclareLaunchArgument("nav_cmd_output_topic", default_value="/nav_cmd"),
        DeclareLaunchArgument("publish_rate_hz", default_value="20.0"),
        DeclareLaunchArgument("cmd_vel_timeout_sec", default_value="0.25"),
        DeclareLaunchArgument("goal_reached_latch_sec", default_value="1.0"),
        DeclareLaunchArgument("force_zero_angular_z", default_value="true"),
        DeclareLaunchArgument("enable_heading_align", default_value="false"),
        DeclareLaunchArgument("heading_align_only_when_force_zero_angular_z", default_value="true"),
        DeclareLaunchArgument("heading_align_min_speed", default_value="0.15"),
        DeclareLaunchArgument("heading_align_kp", default_value="1.2"),
        DeclareLaunchArgument("heading_align_max_wz", default_value="0.5"),
        DeclareLaunchArgument("heading_align_deadband_rad", default_value="0.08"),
        DeclareLaunchArgument("disable_heading_align_when_reached", default_value="true"),
        DeclareLaunchArgument("invert_linear_y", default_value="false"),
        DeclareLaunchArgument("max_linear_x", default_value="2.0"),
        DeclareLaunchArgument("max_linear_y", default_value="2.0"),
        DeclareLaunchArgument("max_linear_speed", default_value="2.0"),
        DeclareLaunchArgument("max_angular_z", default_value="0.5"),
        DeclareLaunchArgument("max_linear_accel", default_value="2.5"),
        DeclareLaunchArgument("max_angular_accel", default_value="1.0"),
        DeclareLaunchArgument("log_output_hz", default_value="1.0"),
        DeclareLaunchArgument("publish_livox_frame_alias", default_value="false"),
        DeclareLaunchArgument("lidar_x", default_value="0.0"),
        DeclareLaunchArgument("lidar_y", default_value="0.2"),
        DeclareLaunchArgument("lidar_z", default_value="0.35"),
        DeclareLaunchArgument("lidar_roll", default_value="0.0"),
        DeclareLaunchArgument("lidar_pitch", default_value="0.3115"),
        DeclareLaunchArgument("lidar_yaw", default_value="1.5708"),
        DeclareLaunchArgument("enable_second_lidar", default_value="false"),
        DeclareLaunchArgument("enable_second_lidar_filter", default_value="true"),
        DeclareLaunchArgument("enable_second_lidar_costmap", default_value="false"),
        DeclareLaunchArgument("enable_second_lidar_safety_limiter", default_value="true"),
        DeclareLaunchArgument("second_lidar_x", default_value="0.0"),
        DeclareLaunchArgument("second_lidar_y", default_value="-0.2"),
        DeclareLaunchArgument("second_lidar_z", default_value="0.35"),
        DeclareLaunchArgument("second_lidar_roll", default_value="0.0"),
        DeclareLaunchArgument("second_lidar_pitch", default_value="0.3115"),
        DeclareLaunchArgument("second_lidar_yaw", default_value="-1.5708"),
        DeclareLaunchArgument("second_lidar_frame", default_value="second_mid360"),
        DeclareLaunchArgument("second_lidar_filter_target_frame", default_value="gimbal_yaw"),
        DeclareLaunchArgument("second_lidar_safety_input_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("second_lidar_safety_output_topic", default_value="/cmd_vel_safe"),

        OpaqueFunction(function=_validate_paths),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="pb_base_footprint_to_base_link",
            output="screen",
            additional_env=navigation_env,
            arguments=[
                "--x", "0.0",
                "--y", "0.0",
                "--z", "0.0",
                "--roll", "0.0",
                "--pitch", "0.0",
                "--yaw", "0.0",
                "--frame-id", "base_footprint",
                "--child-frame-id", "base_link",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="pb_base_footprint_to_gimbal_yaw",
            output="screen",
            additional_env=navigation_env,
            arguments=[
                "--x", "0.0",
                "--y", "0.0",
                "--z", "0.0",
                "--roll", "0.0",
                "--pitch", "0.0",
                "--yaw", "0.0",
                "--frame-id", "base_footprint",
                "--child-frame-id", "gimbal_yaw",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="pb_base_link_to_front_mid360",
            output="screen",
            additional_env=navigation_env,
            arguments=[
                "--x", LaunchConfiguration("lidar_x"),
                "--y", LaunchConfiguration("lidar_y"),
                "--z", LaunchConfiguration("lidar_z"),
                "--roll", LaunchConfiguration("lidar_roll"),
                "--pitch", LaunchConfiguration("lidar_pitch"),
                "--yaw", LaunchConfiguration("lidar_yaw"),
                "--frame-id", "base_link",
                "--child-frame-id", "front_mid360",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="pb_front_mid360_to_livox_frame_alias",
            output="screen",
            condition=IfCondition(_is_true("publish_livox_frame_alias")),
            additional_env=navigation_env,
            arguments=[
                "--x", "0.0",
                "--y", "0.0",
                "--z", "0.0",
                "--roll", "0.0",
                "--pitch", "0.0",
                "--yaw", "0.0",
                "--frame-id", "front_mid360",
                "--child-frame-id", "livox_frame",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="pb_base_link_to_second_mid360",
            output="screen",
            condition=IfCondition(second_lidar_enabled),
            additional_env=navigation_env,
            arguments=[
                "--x", LaunchConfiguration("second_lidar_x"),
                "--y", LaunchConfiguration("second_lidar_y"),
                "--z", LaunchConfiguration("second_lidar_z"),
                "--roll", LaunchConfiguration("second_lidar_roll"),
                "--pitch", LaunchConfiguration("second_lidar_pitch"),
                "--yaw", LaunchConfiguration("second_lidar_yaw"),
                "--frame-id", "base_link",
                "--child-frame-id", LaunchConfiguration("second_lidar_frame"),
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="pb_static_map_to_odom",
            output="screen",
            condition=IfCondition(static_map_to_odom_enabled),
            additional_env=navigation_env,
            arguments=[
                "--x", "0.0",
                "--y", "0.0",
                "--z", "0.0",
                "--roll", "0.0",
                "--pitch", "0.0",
                "--yaw", "0.0",
                "--frame-id", "map",
                "--child-frame-id", "odom",
            ],
        ),

        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="livox_ros_driver2",
            output="screen",
            namespace=namespace,
            condition=IfCondition(_is_false("enable_second_lidar")),
            additional_env=navigation_env,
            parameters=[configured_params],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="livox_ros_driver2",
            output="screen",
            namespace=namespace,
            condition=IfCondition(second_lidar_enabled),
            additional_env=navigation_env,
            parameters=[configured_params, {
                "xfer_format": 1,
                "multi_topic": 1,
                "data_src": 0,
                "publish_freq": 20.0,
                "output_data_type": 0,
                "frame_id": "front_mid360",
                "user_config_path": os.path.join(
                    bringup_dir, "config", "pb2025_xmu_dual_mid360_config.json"
                ),
                "cmdline_input_bd_code": "livox0000000001",
                "lvx_file_path": "",
            }],
            remappings=[
                ("livox/lidar_192_168_1_173", "/livox/lidar"),
                ("livox/imu_192_168_1_173", "/livox/imu"),
                ("livox/lidar_192_168_1_166", "/second_livox/custom_lidar"),
                ("livox/imu_192_168_1_166", "/second_livox/imu"),
            ],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="rm_bringup",
            executable="livox_custom_to_pointcloud2.py",
            name="front_livox_custom_to_pointcloud2",
            output="screen",
            condition=IfCondition(second_lidar_enabled),
            additional_env=navigation_env,
            parameters=[{
                "input_topic": "/livox/lidar",
                "output_topic": "/livox/lidar/pointcloud",
                "frame_id": "front_mid360",
                "use_sim_time": use_sim_time,
            }],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="rm_bringup",
            executable="livox_custom_to_pointcloud2.py",
            name="second_livox_custom_to_pointcloud2",
            output="screen",
            condition=IfCondition(second_lidar_enabled),
            additional_env=navigation_env,
            parameters=[{
                "input_topic": "/second_livox/custom_lidar",
                "output_topic": "/second_livox/lidar",
                "frame_id": LaunchConfiguration("second_lidar_frame"),
                "use_sim_time": use_sim_time,
            }],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="rm_bringup",
            executable="second_lidar_obstacle_filter.py",
            name="second_lidar_obstacle_filter",
            output="screen",
            condition=IfCondition(second_lidar_filter_enabled),
            additional_env=navigation_env,
            parameters=[{
                "input_topic": "/second_livox/lidar",
                "output_topic": "/second_lidar_obstacle_cloud",
                "debug_topic": "/second_lidar_obstacle_debug",
                "target_frame": LaunchConfiguration("second_lidar_filter_target_frame"),
                "source_frame": LaunchConfiguration("second_lidar_frame"),
                "use_sim_time": use_sim_time,
            }],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="rm_bringup",
            executable="second_lidar_safety_limiter_node",
            name="second_lidar_safety_limiter",
            output="screen",
            condition=IfCondition(second_lidar_safety_enabled),
            additional_env=navigation_env,
            parameters=[{
                "input_cmd_vel_topic": LaunchConfiguration("second_lidar_safety_input_topic"),
                "output_cmd_vel_topic": LaunchConfiguration("second_lidar_safety_output_topic"),
                "obstacle_topic": "/second_lidar_obstacle_cloud",
                "debug_topic": "/second_lidar_safety_debug",
                "frame_id": LaunchConfiguration("second_lidar_filter_target_frame"),
                "use_sim_time": use_sim_time,
            }],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="point_lio",
            executable="pointlio_mapping",
            name="point_lio",
            output="screen",
            namespace=namespace,
            respawn=use_respawn,
            respawn_delay=2.0,
            additional_env=navigation_env,
            parameters=[
                configured_params,
                {"prior_pcd.prior_pcd_map_path": prior_pcd_file},
                {"prior_pcd.enable": LaunchConfiguration("enable_prior_pcd")},
                {"pcd_save.pcd_save_en": slam},
            ],
            arguments=["--ros-args", "--log-level", log_level],
        ),

        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            namespace=namespace,
            condition=IfCondition(no_slam),
            respawn=use_respawn,
            respawn_delay=2.0,
            additional_env=navigation_env,
            parameters=[configured_params],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="small_gicp_relocalization",
            executable="small_gicp_relocalization_node",
            name="small_gicp_relocalization",
            output="screen",
            namespace=namespace,
            condition=IfCondition(small_gicp_enabled),
            respawn=use_respawn,
            respawn_delay=2.0,
            additional_env=navigation_env,
            parameters=[configured_params, {"prior_pcd_file": prior_pcd_file}],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            namespace=namespace,
            condition=IfCondition(no_slam),
            additional_env=navigation_env,
            parameters=[
                {"use_sim_time": use_sim_time},
                {"autostart": autostart},
                {"node_names": ["map_server"]},
            ],
            arguments=["--ros-args", "--log-level", log_level],
        ),

        Node(
            package="nav2_map_server",
            executable="map_saver_server",
            name="map_saver",
            output="screen",
            namespace=namespace,
            condition=IfCondition(use_slam),
            respawn=use_respawn,
            respawn_delay=2.0,
            additional_env=navigation_env,
            parameters=[configured_params],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_slam",
            output="screen",
            namespace=namespace,
            condition=IfCondition(use_slam),
            additional_env=navigation_env,
            parameters=[
                {"use_sim_time": use_sim_time},
                {"autostart": autostart},
                {"node_names": ["map_saver"]},
            ],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan",
            output="screen",
            namespace=namespace,
            condition=IfCondition(use_slam),
            respawn=use_respawn,
            respawn_delay=2.0,
            additional_env=navigation_env,
            parameters=[configured_params],
            remappings=[("cloud_in", "terrain_map_ext"), ("scan", "obstacle_scan")],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="slam_toolbox",
            executable="sync_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            namespace=namespace,
            condition=IfCondition(use_slam),
            respawn=use_respawn,
            respawn_delay=2.0,
            additional_env=navigation_env,
            parameters=[configured_params],
            remappings=[("/map", "map"), ("/map_metadata", "map_metadata"), ("/map_updates", "map_updates")],
            arguments=["--ros-args", "--log-level", log_level],
        ),

        Node(
            package="loam_interface",
            executable="loam_interface_node",
            name="loam_interface",
            output="screen",
            namespace=namespace,
            respawn=use_respawn,
            respawn_delay=2.0,
            additional_env=navigation_env,
            parameters=[configured_params],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="sensor_scan_generation",
            executable="sensor_scan_generation_node",
            name="sensor_scan_generation",
            output="screen",
            namespace=namespace,
            respawn=use_respawn,
            respawn_delay=2.0,
            additional_env=navigation_env,
            parameters=[configured_params],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="terrain_analysis",
            executable="terrainAnalysis",
            name="terrain_analysis",
            output="screen",
            namespace=namespace,
            respawn=use_respawn,
            respawn_delay=2.0,
            additional_env=navigation_env,
            parameters=[configured_params],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="terrain_analysis_ext",
            executable="terrainAnalysisExt",
            name="terrain_analysis_ext",
            output="screen",
            namespace=namespace,
            respawn=use_respawn,
            respawn_delay=2.0,
            additional_env=navigation_env,
            parameters=[configured_params],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="fake_vel_transform",
            executable="fake_vel_transform_node",
            name="fake_vel_transform",
            output="screen",
            namespace=namespace,
            respawn=use_respawn,
            respawn_delay=2.0,
            additional_env=navigation_env,
            parameters=[configured_params],
            arguments=["--ros-args", "--log-level", log_level],
        ),

        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            namespace=namespace,
            respawn=use_respawn,
            respawn_delay=2.0,
            additional_env=navigation_env,
            parameters=[configured_params],
            remappings=[("cmd_vel", "cmd_vel_controller")],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            output="screen",
            namespace=namespace,
            respawn=use_respawn,
            respawn_delay=2.0,
            additional_env=navigation_env,
            parameters=[configured_params],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            namespace=namespace,
            respawn=use_respawn,
            respawn_delay=2.0,
            additional_env=navigation_env,
            parameters=[configured_params],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            namespace=namespace,
            respawn=use_respawn,
            respawn_delay=2.0,
            additional_env=navigation_env,
            parameters=[configured_params],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            namespace=namespace,
            respawn=use_respawn,
            respawn_delay=2.0,
            additional_env=navigation_env,
            parameters=[configured_params],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            namespace=namespace,
            respawn=use_respawn,
            respawn_delay=2.0,
            additional_env=navigation_env,
            parameters=[configured_params],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            namespace=namespace,
            respawn=use_respawn,
            respawn_delay=2.0,
            additional_env=navigation_env,
            parameters=[configured_params],
            remappings=[
                ("cmd_vel", "cmd_vel_controller"),
                ("cmd_vel_smoothed", "cmd_vel_nav2_result"),
            ],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            namespace=namespace,
            additional_env=navigation_env,
            parameters=[
                {"use_sim_time": use_sim_time},
                {"autostart": autostart},
                {
                    "node_names": [
                        "controller_server",
                        "smoother_server",
                        "planner_server",
                        "behavior_server",
                        "bt_navigator",
                        "waypoint_follower",
                        "velocity_smoother",
                    ],
                },
            ],
            arguments=["--ros-args", "--log-level", log_level],
        ),

        Node(
            package="rm_bringup",
            executable="pb_cmd_vel_to_nav_cmd.py",
            name="pb_cmd_vel_to_nav_cmd",
            output="screen",
            condition=IfCondition(cmd_bridge_safe_enabled),
            additional_env=navigation_env,
            parameters=[{
                "input_topic": LaunchConfiguration("cmd_vel_safe_topic"),
                "output_topic": LaunchConfiguration("nav_cmd_output_topic"),
                "cmd_vel_timeout_sec": LaunchConfiguration("cmd_vel_timeout_sec"),
                "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
                "goal_reached_latch_sec": LaunchConfiguration("goal_reached_latch_sec"),
                "force_zero_angular_z": LaunchConfiguration("force_zero_angular_z"),
                "enable_heading_align": LaunchConfiguration("enable_heading_align"),
                "heading_align_only_when_force_zero_angular_z": LaunchConfiguration("heading_align_only_when_force_zero_angular_z"),
                "heading_align_min_speed": LaunchConfiguration("heading_align_min_speed"),
                "heading_align_kp": LaunchConfiguration("heading_align_kp"),
                "heading_align_max_wz": LaunchConfiguration("heading_align_max_wz"),
                "heading_align_deadband_rad": LaunchConfiguration("heading_align_deadband_rad"),
                "disable_heading_align_when_reached": LaunchConfiguration("disable_heading_align_when_reached"),
                "invert_linear_y": LaunchConfiguration("invert_linear_y"),
                "max_linear_x": LaunchConfiguration("max_linear_x"),
                "max_linear_y": LaunchConfiguration("max_linear_y"),
                "max_linear_speed": LaunchConfiguration("max_linear_speed"),
                "max_angular_z": LaunchConfiguration("max_angular_z"),
                "max_linear_accel": LaunchConfiguration("max_linear_accel"),
                "max_angular_accel": LaunchConfiguration("max_angular_accel"),
                "log_output_hz": LaunchConfiguration("log_output_hz"),
                "use_sim_time": use_sim_time,
            }],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="rm_bringup",
            executable="pb_cmd_vel_to_nav_cmd.py",
            name="pb_cmd_vel_to_nav_cmd",
            output="screen",
            condition=IfCondition(cmd_bridge_direct_enabled),
            additional_env=navigation_env,
            parameters=[{
                "input_topic": LaunchConfiguration("cmd_vel_input_topic"),
                "output_topic": LaunchConfiguration("nav_cmd_output_topic"),
                "cmd_vel_timeout_sec": LaunchConfiguration("cmd_vel_timeout_sec"),
                "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
                "goal_reached_latch_sec": LaunchConfiguration("goal_reached_latch_sec"),
                "force_zero_angular_z": LaunchConfiguration("force_zero_angular_z"),
                "enable_heading_align": LaunchConfiguration("enable_heading_align"),
                "heading_align_only_when_force_zero_angular_z": LaunchConfiguration("heading_align_only_when_force_zero_angular_z"),
                "heading_align_min_speed": LaunchConfiguration("heading_align_min_speed"),
                "heading_align_kp": LaunchConfiguration("heading_align_kp"),
                "heading_align_max_wz": LaunchConfiguration("heading_align_max_wz"),
                "heading_align_deadband_rad": LaunchConfiguration("heading_align_deadband_rad"),
                "disable_heading_align_when_reached": LaunchConfiguration("disable_heading_align_when_reached"),
                "invert_linear_y": LaunchConfiguration("invert_linear_y"),
                "max_linear_x": LaunchConfiguration("max_linear_x"),
                "max_linear_y": LaunchConfiguration("max_linear_y"),
                "max_linear_speed": LaunchConfiguration("max_linear_speed"),
                "max_angular_z": LaunchConfiguration("max_angular_z"),
                "max_linear_accel": LaunchConfiguration("max_linear_accel"),
                "max_angular_accel": LaunchConfiguration("max_angular_accel"),
                "log_output_hz": LaunchConfiguration("log_output_hz"),
                "use_sim_time": use_sim_time,
            }],
            arguments=["--ros-args", "--log-level", log_level],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="pb2025_nav_rviz",
            output="screen",
            condition=IfCondition(rviz_enabled),
            additional_env=navigation_env,
            arguments=[
                "-d",
                PathJoinSubstitution([
                    FindPackageShare("pb2025_nav_bringup"),
                    "rviz",
                    "nav2_default_view.rviz",
                ]),
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
