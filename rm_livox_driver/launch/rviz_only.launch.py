"""
rm_livox_driver / rviz_only.launch.py
======================================
Launch ONLY RViz2 with the MID-360 pointcloud display config.

Use this for:
  - Visual debugging when driver is already running in another terminal
  - Checking TF tree and topic data interactively

Usage:
  ros2 launch rm_livox_driver rviz_only.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rm_livox_driver')
    rviz_config = os.path.join(pkg_share, 'config', 'mid360_view.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='livox_rviz',
        output='screen',
        arguments=['--display-config', rviz_config],
    )

    return LaunchDescription([
        rviz_node,
    ])