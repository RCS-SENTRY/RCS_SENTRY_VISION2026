"""
rm_livox_driver / lidar_only.launch.py
======================================
Launch ONLY the Livox MID-360 driver node. No TF, no RViz.

Use this for:
  - Verifying LiDAR connectivity (check `ros2 topic echo /livox/lidar`)
  - Debugging network configuration
  - Feeding data to FAST-LIO / Point-LIO separately

Usage:
  ros2 launch rm_livox_driver lidar_only.launch.py
  ros2 launch rm_livox_driver lidar_only.launch.py xfer_format:=1
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rm_livox_driver')
    config_dir = os.path.join(pkg_share, 'config')

    xfer_format_arg = DeclareLaunchArgument(
        'xfer_format', default_value='0',
        description='0=PointCloud2, 1=CustomMsg (for FAST-LIO)'
    )
    publish_freq_arg = DeclareLaunchArgument(
        'publish_freq', default_value='20.0',
        description='LiDAR publish frequency'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock (true for Gazebo, false for real hardware)'
    )

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format':           LaunchConfiguration('xfer_format'),
            'multi_topic':           0,
            'data_src':              0,
            'publish_freq':          LaunchConfiguration('publish_freq'),
            'output_data_type':      0,
            'frame_id':              'livox_frame',
            'lvx_file_path':         '/home/livox/livox_test.lvx',
            'user_config_path':      os.path.join(config_dir, 'MID360_config.json'),
            'cmdline_input_bd_code': 'livox0000000001',
            'use_sim_time':          LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([
        xfer_format_arg,
        publish_freq_arg,
        use_sim_time_arg,
        livox_driver,
    ])