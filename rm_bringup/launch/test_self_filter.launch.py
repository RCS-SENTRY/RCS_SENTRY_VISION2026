"""
test_self_filter.launch.py — 无雷达无串口测试自车剔除链路

启动内容:
  1. static TF: base_link -> livox_frame (与实车一致)
  2. fake_lidar_publisher: 模拟 Livox 点云 (含自车点云)
  3. self_point_filter_node: 自车剔除 (CustomMsg 链)
  4. obstacle_cloud_filter_node: Nav2 障碍过滤 (PointCloud2 链)

验证方法:
  # 终端1: 启动测试
  ros2 launch rm_bringup test_self_filter.launch.py

  # 终端2: 对比点云数量
  ros2 topic echo /livox/lidar --field point_num --once         # 原始点数
  ros2 topic echo /livox/lidar/self_filtered --field point_num --once  # 剔除后点数
  ros2 topic echo /nav_obstacle_cloud --field width --once      # Nav2 障碍点数

  # 终端3: RViz 可视化
  rviz2  # 添加 PointCloud2 显示, 分别订阅三个 topic
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('rm_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ---- Static TF: base_link -> livox_frame (与实车参数一致) ----
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_livox_frame',
        arguments=[
            '--x', '0.0',
            '--y', '0.2',
            '--z', '0.35',
            '--roll', '0.0',
            '--pitch', '0.3115',
            '--yaw', '1.5708',
            '--frame-id', 'base_link',
            '--child-frame-id', 'livox_frame',
        ],
    )

    # ---- Fake LiDAR (模拟 Livox MID-360 点云) ----
    fake_lidar = Node(
        package='rm_bringup',
        executable='fake_lidar_publisher.py',
        name='fake_lidar_publisher',
        output='screen',
    )

    # ---- Self-point filter (建图链: CustomMsg 自车剔除) ----
    self_filter = Node(
        package='rm_bringup',
        executable='self_point_filter_node',
        name='self_point_filter',
        output='screen',
        parameters=[{
            'input_topic': '/livox/lidar',
            'output_topic': '/livox/lidar/self_filtered',
            'source_frame': 'livox_frame',
            'target_frame': 'base_link',
            'transform_timeout_sec': 0.05,
            'use_sim_time': False,
        }],
    )

    # ---- Obstacle cloud filter (Nav2链: PointCloud2 自车剔除 + 障碍过滤) ----
    obstacle_filter = Node(
        package='rm_bringup',
        executable='obstacle_cloud_filter_node',
        name='obstacle_cloud_filter',
        output='screen',
        parameters=[{
            'primary_input_topic': '/livox/lidar/pointcloud',
            'secondary_input_topic': '',
            'output_topic': '/nav_obstacle_cloud',
            'target_frame': 'base_link',
            'min_height': 0.05,
            'max_height': 1.50,
            'min_range': 0.20,
            'max_range': 5.00,
            'voxel_leaf_size': 0.10,
            'merge_timeout_sec': 0.20,
            'transform_timeout_sec': 0.05,
            'body_box.enabled': True,
            'body_box.min': [-0.30, -0.30, -0.10],
            'body_box.max': [0.30, 0.30, 0.60],
            'gimbal_box.enabled': True,
            'gimbal_box.min': [-0.15, -0.15, 0.40],
            'gimbal_box.max': [0.15, 0.15, 0.80],
            'gimbal_support_box.enabled': True,
            'gimbal_support_box.min': [-0.08, -0.08, 0.25],
            'gimbal_support_box.max': [0.08, 0.08, 0.45],
            'lidar_arm_box.enabled': True,
            'lidar_arm_box.min': [-0.05, 0.10, 0.20],
            'lidar_arm_box.max': [0.10, 0.35, 0.50],
            'use_sim_time': False,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # 立即启动 TF
        static_tf,
        # 延迟 0.5s 启动 fake lidar (等 TF 就绪)
        TimerAction(period=0.5, actions=[fake_lidar]),
        # 延迟 0.3s 启动两个 filter (等 fake lidar 开始发布)
        TimerAction(period=0.3, actions=[self_filter]),
        TimerAction(period=0.3, actions=[obstacle_filter]),
    ])