import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rm_global_localization')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    declare_map_path = DeclareLaunchArgument(
        'map_path',
        default_value='/home/rm/Desktop/SENTRY_FULL/RMUC2026.pcd',
        description='Absolute path to the static PCD map')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock')

    declare_enable_reset_odom_on_recovery = DeclareLaunchArgument(
        'enable_reset_odom_on_recovery',
        default_value='true',
        description='Whether to call /reset_odom after recovery')

    node = Node(
        package='rm_global_localization',
        executable='global_localization_node',
        name='rm_global_localization',
        output='screen',
        parameters=[
            params_file,
            {
                'map_path': LaunchConfiguration('map_path'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'enable_reset_odom_on_recovery': LaunchConfiguration('enable_reset_odom_on_recovery'),
            },
        ],
    )

    return LaunchDescription([
        declare_map_path,
        declare_use_sim_time,
        declare_enable_reset_odom_on_recovery,
        node,
    ])
