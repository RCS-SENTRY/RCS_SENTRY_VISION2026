"""
rm_livox_driver / mid360_bringup.launch.py
==========================================
Clean bringup for Livox MID-360 LiDAR.

Architecture rules (from pathology report):
  1. NO middleware pointcloud forwarding -- we launch livox_ros_driver2 directly.
  2. frame_id is standardized to 'livox_frame' via parameter.
     NOTE: livox_ros_driver2 hardcodes IMU frame_id to 'livox_frame' internally,
     so our pointcloud frame_id MUST match -- this is the canonical name.
  3. Static TF is NOT published here. It must be provided by the system launch.
  4. Default xfer_format=4 so navigation and visualization can coexist:
       - /livox/lidar            (livox_ros_driver2/msg/CustomMsg)   for Point-LIO
       - /livox/lidar/pointcloud (sensor_msgs/msg/PointCloud2)       for RViz

Topics published:
  /livox/lidar             -- livox_ros_driver2/msg/CustomMsg (frame: livox_frame)
  /livox/lidar/pointcloud  -- sensor_msgs/PointCloud2         (frame: livox_frame)
  /livox/imu               -- sensor_msgs/Imu                 (frame: livox_frame)

Usage:
  ros2 launch rm_livox_driver mid360_bringup.launch.py
  ros2 launch rm_livox_driver mid360_bringup.launch.py rviz:=false
  ros2 launch rm_livox_driver mid360_bringup.launch.py xfer_format:=4
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # -- Paths --
    pkg_share = get_package_share_directory('rm_livox_driver')
    config_dir = os.path.join(pkg_share, 'config')

    # -- Launch arguments (user-configurable) --
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Whether to launch RViz2 for pointcloud visualization'
    )
    xfer_format_arg = DeclareLaunchArgument(
        'xfer_format', default_value='4',
        description='0=PointCloud2 only, 1=CustomMsg only, 4=CustomMsg + PointCloud2 (recommended)'
    )
    publish_freq_arg = DeclareLaunchArgument(
        'publish_freq', default_value='20.0',
        description='LiDAR publish frequency (5.0, 10.0, 20.0, 50.0)'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock (true for Gazebo, false for real hardware)'
    )
    # -- Livox driver node --
    # xfer_format=4 -> /livox/lidar (CustomMsg) + /livox/lidar/pointcloud (PointCloud2)
    #                  This is the recommended mode for RM navigation QA.
    # xfer_format=0 -> PointCloud2 only (RViz direct view, Point-LIO will NOT receive AVIA data)
    # xfer_format=1 -> CustomMsg only  (Point-LIO works, but RViz needs another topic source)
    # frame_id is forced to 'livox_frame' -- matches the hardcoded IMU frame_id.
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

    # -- RViz2 (conditional) --
    rviz_config = os.path.join(config_dir, 'mid360_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='livox_rviz',
        output='screen',
        arguments=['--display-config', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        # Launch arguments
        rviz_arg,
        xfer_format_arg,
        publish_freq_arg,
        use_sim_time_arg,
        # Nodes
        livox_driver,
        rviz_node,
    ])
