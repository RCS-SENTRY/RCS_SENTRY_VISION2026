import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _validate_inputs(context, *args, **kwargs):
    map_yaml = LaunchConfiguration("map").perform(context)
    params_file = LaunchConfiguration("params_file").perform(context)

    if not map_yaml:
        raise RuntimeError(
            "pure_navigation_bringup.launch.py 需要显式传入二维占据栅格地图: map:=/abs/path/to/map.yaml"
        )
    if not os.path.exists(map_yaml):
        raise RuntimeError(f"找不到地图文件: {map_yaml}")
    if not os.path.exists(params_file):
        raise RuntimeError(f"找不到 Nav2 参数文件: {params_file}")
    return []


def generate_launch_description():
    bringup_dir = get_package_share_directory("rm_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")
    log_level = LaunchConfiguration("log_level")

    declare_map = DeclareLaunchArgument(
        "map",
        default_value="",
        description="二维占据栅格地图 yaml 绝对路径（供 map_server / global_costmap 使用）",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="是否使用仿真时钟",
    )
    declare_autostart = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="是否自动激活 Nav2 生命周期节点",
    )
    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "config", "sentry_nav2_params.yaml"),
        description="Nav2 参数文件路径",
    )
    declare_log_level = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="日志等级",
    )

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    lifecycle_nodes = [
        "map_server",
        "planner_server",
        "controller_server",
        "behavior_server",
        "bt_navigator",
    ]

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time, "yaml_filename": map_yaml}],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=remappings,
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=remappings,
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=remappings,
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=remappings,
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=remappings,
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": lifecycle_nodes},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # 纯协议桥：只把 Nav2 的 geometry_msgs/Twist 翻译成现有 /nav_cmd 契约
    cmd_vel_bridge = Node(
        package="rm_bringup",
        executable="cmd_vel_to_nav_cmd.py",
        name="cmd_vel_to_nav_cmd",
        output="screen",
        parameters=[
            {"input_topic": "/cmd_vel"},
            {"output_topic": "/nav_cmd"},
            {"cmd_vel_timeout_sec": 0.25},
            {"publish_rate_hz": 20.0},
            {"use_sim_time": use_sim_time},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            declare_map,
            declare_use_sim_time,
            declare_autostart,
            declare_params_file,
            declare_log_level,
            OpaqueFunction(function=_validate_inputs),
            map_server,
            planner_server,
            controller_server,
            behavior_server,
            bt_navigator,
            lifecycle_manager,
            cmd_vel_bridge,
        ]
    )
