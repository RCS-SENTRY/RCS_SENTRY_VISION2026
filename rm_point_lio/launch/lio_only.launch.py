"""
rm_point_lio / lio_only.launch.py
==================================
Launch ONLY the Point-LIO node. No driver, no TF, no RViz.

Use this for:
  - Debugging Point-LIO with pre-recorded rosbag
  - Testing parameter changes in isolation
  - Feeding data from rm_livox_driver running in another terminal

Prerequisites:
  - rm_livox_driver or rosbag must be publishing /livox/lidar and /livox/imu

Usage:
  ros2 launch rm_point_lio lio_only.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rm_point_lio')
    config_file = os.path.join(pkg_share, 'config', 'mid360_point_lio.yaml')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock (true for rosbag with --clock)'
    )

    pointlio_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=[
            config_file,
            {
                'odom_header_frame_id': 'odom',
                'odom_child_frame_id':  'base_link',
                'use_imu_as_input': False,
                'prop_at_freq_of_imu': True,
                'check_satu': True,
                'init_map_size': 10,
                'point_filter_num': 3,
                'space_down_sample': True,
                'filter_size_surf': 0.5,
                'filter_size_map': 0.5,
                'cube_side_length': 1000.0,
                'runtime_pos_log_enable': False,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        pointlio_node,
    ])