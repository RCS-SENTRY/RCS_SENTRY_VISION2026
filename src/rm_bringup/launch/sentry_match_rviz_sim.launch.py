import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("rm_bringup")
    sentry_bt_dir = get_package_share_directory("sentry_bt")
    sentry_decision_dir = get_package_share_directory("rm_sentry_decision")

    map_file = LaunchConfiguration("map")
    goals_file = LaunchConfiguration("goals_file")
    scenario = LaunchConfiguration("scenario")
    scenario_path = PathJoinSubstitution([sentry_bt_dir, scenario])
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription([
        DeclareLaunchArgument(
            "map",
            default_value="/home/rm/Desktop/SENTRY_FULL/maps/new_map.yaml",
        ),
        DeclareLaunchArgument(
            "goals_file",
            default_value=os.path.join(sentry_decision_dir, "config", "sentry_goals.yaml"),
        ),
        DeclareLaunchArgument("scenario", default_value="match_visual_full.yaml"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(
                bringup_dir, "config", "sentry_match_sim.rviz"),
        ),
        DeclareLaunchArgument("initial_x", default_value="0.0"),
        DeclareLaunchArgument("initial_y", default_value="0.0"),
        DeclareLaunchArgument("initial_yaw", default_value="2.15"),
        DeclareLaunchArgument("sim_speed_mps", default_value="1.2"),
        DeclareLaunchArgument("goal_active_timeout_sec", default_value="10.0"),

        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{"yaml_filename": map_file, "use_sim_time": False}],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_match_sim",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "autostart": True,
                "node_names": ["map_server"],
            }],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="sim_gimbal_to_base_link",
            output="screen",
            arguments=[
                "--x", "0.0", "--y", "0.0", "--z", "0.0",
                "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                "--frame-id", "gimbal_yaw_fake",
                "--child-frame-id", "base_link",
            ],
        ),
        Node(
            package="rm_bringup",
            executable="sentry_match_nav_sim.py",
            name="sentry_match_nav_sim",
            output="screen",
            parameters=[{
                "goals_file": goals_file,
                "initial_x": LaunchConfiguration("initial_x"),
                "initial_y": LaunchConfiguration("initial_y"),
                "initial_yaw": LaunchConfiguration("initial_yaw"),
                "speed_mps": LaunchConfiguration("sim_speed_mps"),
                "navigate_action_name": "navigate_to_pose",
                "scenario_path": scenario_path,
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sentry_bt_dir, "sentry_bt.launch.py")),
            launch_arguments={
                "mode": "debug",
                "scenario": scenario,
                "show_debug_watch": "true",
                "watch_view": "io",
                "params_file": os.path.join(sentry_bt_dir, "sentry_bt_params.yaml"),
            }.items(),
        ),
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package="rm_sentry_decision",
                    executable="sentry_goal_executor_node",
                    name="sentry_goal_executor",
                    output="screen",
                    parameters=[{
                        "goals_file": goals_file,
                        "intent_topic": "/sentry/intent",
                        "nav_status_topic": "/sentry/nav_status",
                        "goal_active_timeout_sec": LaunchConfiguration("goal_active_timeout_sec"),
                        "navigate_action_name": "navigate_to_pose",
                        "robot_base_frame": "gimbal_yaw_fake",
                    }],
                ),
                Node(
                    package="rm_sentry_decision",
                    executable="sentry_command_mux_node",
                    name="sentry_command_mux",
                    output="screen",
                    parameters=[{
                        "intent_topic": "/sentry/intent",
                        "gimbal_status_topic": "/gimbal_status",
                        "autoaim_raw_cmd_topic": "/autoaim/gimbal_cmd_raw",
                        "final_gimbal_cmd_topic": "/gimbal_cmd",
                        "intent_timeout_sec": 1.0,
                        "enable_intent_only_heartbeat": True,
                    }],
                ),
            ],
        ),
        Node(
            condition=IfCondition(use_rviz),
            package="rviz2",
            executable="rviz2",
            name="rviz2_match_sim",
            output="screen",
            arguments=["-d", rviz_config],
        ),
    ])
