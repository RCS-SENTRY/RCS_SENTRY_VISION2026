"""
rm_point_lio / point_lio.launch.py
===================================
Clean wrapper launch for Point-LIO with MID-360 on RM Sentry.

Architecture: Wrapper Isolation Principle
  - This package does NOT modify point_lio source code.
  - All customization is done via parameter override.
  - We launch point_lio's node directly with our config.

Data flow:
  /livox/lidar/self_filtered (CustomMsg, from self_point_filter_node — 自车已剔除)
  /livox/imu                 (Imu from rm_livox_driver)
       |
       v
  pointlio_mapping (point_lio)
       |
       +-- /cloud_registered  (PointCloud2, frame: odom)
       +-- /Odometry          (nav_msgs/Odometry, header: odom, child: base_link)
       +-- /path              (nav_msgs/Path, frame: odom)

TF tree after this launch:
  odom (published by Point-LIO via /Odometry)
    +-- base_link (published by Point-LIO via /Odometry)

Required external TF:
  base_link -> livox_frame must be published by the system launch / robot description.

Coordinate frame contract:
  odom_header_frame_id = "odom"        (global frame, was "camera_init")
  odom_child_frame_id  = "base_link"   (robot base, was "aft_mapped")

Usage:
  ros2 launch rm_point_lio point_lio.launch.py
  ros2 launch rm_point_lio point_lio.launch.py rviz:=false
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
    pkg_share = get_package_share_directory('rm_point_lio')
    config_file = os.path.join(pkg_share, 'config', 'mid360_point_lio.yaml')

    # -- Launch arguments --
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz2 for visualization'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock (true for Gazebo, false for real hardware)'
    )
    # ================================================================
    # Point-LIO core node
    # ================================================================
    # Parameters are loaded in order: YAML file first, then inline overrides.
    # Inline dict takes precedence over YAML for the same key.
    #
    # CRITICAL OVERRIDES (vs point_lio defaults):
    #   odom_header_frame_id: "odom"        (was "camera_init" — non-standard)
    #   odom_child_frame_id:  "base_link"   (was "aft_mapped" — non-standard)
    #   check_satu:           true           (anti-explosion for RM arena)
    #   satu_acc:             30.0           (MID-360 IMU physical range)
    #   satu_gyro:            35.0           (MID-360 IMU physical range)
    laser_mapping_params = [
        config_file,
        {
            # -- Frame ID standardization --
            'odom_header_frame_id': 'odom',
            'odom_child_frame_id':  'base_link',

            # -- Anti-explosion config --
            'use_imu_as_input': False,     # Point-LIO recommended: LiDAR as input
            'prop_at_freq_of_imu': True,
            'check_satu': True,
            'init_map_size': 10,
            'point_filter_num': 3,
            'space_down_sample': True,
            'filter_size_surf': 0.5,
            'filter_size_map': 0.5,
            'cube_side_length': 1000.0,
            'runtime_pos_log_enable': False,

            # -- Sim time --
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }
    ]

    pointlio_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=laser_mapping_params,
    )

    # ================================================================
    # RViz2 (conditional)
    # ================================================================
    rviz_config = os.path.join(pkg_share, 'config', 'point_lio_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='point_lio_rviz',
        output='screen',
        arguments=['--display-config', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        # Arguments
        rviz_arg,
        use_sim_time_arg,
        # Nodes
        pointlio_node,
        rviz_node,
    ])
