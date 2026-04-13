"""
rm_point_lio / lio_with_driver.launch.py
==========================================
Full pipeline: Livox driver (CustomMsg) + Point-LIO + Static TF + RViz.

This is the "one-click" launch for SLAM on the desk / on the robot.
It starts:
  1. livox_ros_driver2_node (xfer_format=1 → CustomMsg for AVIA handler)
  2. pointlio_mapping (with our mid360_point_lio.yaml override)
  3. static TF: base_link → livox_frame
  4. RViz2 with point_lio_view.rviz

Data flow:
  MID-360 hardware
      |  (UDP)
      v
  livox_ros_driver2_node
      |  /livox/lidar (CustomMsg, xfer_format=1, AVIA path)
      |  /livox/imu   (Imu)
      v
  pointlio_mapping (laserMapping)
      |  /cloud_registered  (PointCloud2)
      |  /Odometry          (Odometry: odom → base_link)
      |  /path              (Path)

TF tree:
  odom (via /Odometry)
    +-- base_link (via /Odometry)
          +-- livox_frame (static, z=lidar_z)

Usage:
  ros2 launch rm_point_lio lio_with_driver.launch.py
  ros2 launch rm_point_lio lio_with_driver.launch.py rviz:=false
  ros2 launch rm_point_lio lio_with_driver.launch.py lidar_z:=0.35
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
    pointlio_pkg = get_package_share_directory('rm_point_lio')
    livox_pkg = get_package_share_directory('rm_livox_driver')

    pointlio_config = os.path.join(pointlio_pkg, 'config', 'mid360_point_lio.yaml')
    livox_config = os.path.join(livox_pkg, 'config', 'MID360_config.json')
    rviz_config = os.path.join(pointlio_pkg, 'config', 'point_lio_view.rviz')

    # -- Launch arguments --
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz2 for visualization'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock'
    )
    lidar_z_arg = DeclareLaunchArgument(
        'lidar_z', default_value='0.5',
        description='LiDAR mounting height above base_link (m)'
    )
    publish_freq_arg = DeclareLaunchArgument(
        'publish_freq', default_value='20.0',
        description='LiDAR publish frequency'
    )

    # 1. Livox driver — xfer_format=1 (CustomMsg for Point-LIO AVIA handler)
    #    NOTE: Point-LIO AVIA handler subscribes to CustomMsg via livox_pcl_cbk,
    #    giving better per-point timestamps than standard PointCloud2.
    # ================================================================
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format':           1,  # 1=CustomMsg (required by Point-LIO AVIA handler)
            'multi_topic':           0,
            'data_src':              0,
            'publish_freq':          LaunchConfiguration('publish_freq'),
            'output_data_type':      0,
            'frame_id':              'livox_frame',
            'lvx_file_path':         '/home/livox/livox_test.lvx',
            'user_config_path':      livox_config,
            'cmdline_input_bd_code': 'livox0000000001',
            'use_sim_time':          LaunchConfiguration('use_sim_time'),
        }],
    )

    # ================================================================
    # 2. Point-LIO node
    # ================================================================
    pointlio_node = Node(
        package='point_lio',
        executable='pointlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=[
            pointlio_config,
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

    # ================================================================
    # 3. Static TF: base_link -> livox_frame
    # ================================================================
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_livox_frame',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', LaunchConfiguration('lidar_z'),
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'livox_frame',
        ],
    )

    # ================================================================
    # 4. RViz2 (conditional)
    # ================================================================
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
        rviz_arg,
        use_sim_time_arg,
        lidar_z_arg,
        publish_freq_arg,
        livox_driver,
        pointlio_node,
        static_tf,
        rviz_node,
    ])