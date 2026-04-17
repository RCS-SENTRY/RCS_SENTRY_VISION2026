# =============================================================================
# sentry_bringup.launch.py — RM Sentry 一键启动 (v3 — 全家桶)
# =============================================================================
# 启动顺序:
#   use_serial=true (实车):
#     T+0.0s: rm_hw_bridge (串口)
#     T+2.0s: rm_hik_driver (相机)
#     T+4.0s: rm_vision (检测) + rm_autoaim (自瞄)
#
#   use_serial=false (调试):
#     T+0.0s: rm_hik_driver (相机)
#     T+2.0s: rm_vision (检测) + rm_autoaim (自瞄)
#
# 用法:
#   ros2 launch rm_bringup sentry_bringup.launch.py
#   ros2 launch rm_bringup sentry_bringup.launch.py use_serial:=false
#   ros2 launch rm_bringup sentry_bringup.launch.py enable_decision:=true
#   ros2 launch rm_bringup sentry_bringup.launch.py enable_navigation:=true
#   ros2 launch rm_bringup sentry_bringup.launch.py navigation_only:=true
# =============================================================================
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # ------------------------------------------------------------------
    # 定位各包的参数文件
    # ------------------------------------------------------------------
    hw_bridge_dir  = get_package_share_directory('rm_hw_bridge')
    hik_driver_dir = get_package_share_directory('rm_hik_driver')
    vision_dir     = get_package_share_directory('rm_vision')
    autoaim_dir    = get_package_share_directory('rm_autoaim')
    livox_dir      = get_package_share_directory('rm_livox_driver')
    point_lio_dir  = get_package_share_directory('rm_point_lio')
    global_loc_dir = get_package_share_directory('rm_global_localization')
    bringup_dir    = get_package_share_directory('rm_bringup')
    hw_bridge_params = os.path.join(hw_bridge_dir,  'config', 'params.yaml')
    hik_params       = os.path.join(hik_driver_dir, 'config', 'params.yaml')
    vision_params    = os.path.join(vision_dir,     'config', 'params.yaml')
    autoaim_params   = os.path.join(autoaim_dir,    'config', 'params.yaml')
    livox_launch     = os.path.join(livox_dir,      'launch', 'mid360_bringup.launch.py')
    point_lio_launch = os.path.join(point_lio_dir,  'launch', 'point_lio.launch.py')
    global_loc_launch = os.path.join(global_loc_dir, 'launch', 'global_localization.launch.py')
    initial_pose_params = os.path.join(bringup_dir, 'config', 'initial_pose_manager.yaml')
    pure_nav_launch = os.path.join(bringup_dir, 'launch', 'pure_navigation_bringup.launch.py')

    # ------------------------------------------------------------------
    # Launch 参数声明
    # ------------------------------------------------------------------
    declare_use_serial = DeclareLaunchArgument(
        'use_serial', default_value='true',
        description='true=启动串口(hw_bridge)+延迟相机; false=跳过串口+相机立即启动')

    declare_serial_device = DeclareLaunchArgument(
        'serial_device', default_value='/dev/ttyUSB0',
        description='Serial port device path')

    declare_baudrate = DeclareLaunchArgument(
        'baudrate', default_value='460800',
        description='Serial baudrate')

    declare_color_ignore = DeclareLaunchArgument(
        'color_ignore', default_value='1',
        description='Ignore color: 0=Red, 1=Blue, -1=None')

    declare_target_color = DeclareLaunchArgument(
        'target_color', default_value='red',
        description='Target color: red / blue / all')

    declare_publish_debug = DeclareLaunchArgument(
        'publish_debug_image', default_value='false',
        description='Publish debug image to /detector/image_debug')

    declare_model_path = DeclareLaunchArgument(
        'model_path', default_value='model/yolo11.xml',
        description='Absolute path to OpenVINO model (.xml)')

    declare_enable_decision = DeclareLaunchArgument(
        'enable_decision', default_value='false',
        description='true=启动 sentry_bt 决策节点; false=不启动决策节点')

    declare_enable_navigation = DeclareLaunchArgument(
        'enable_navigation', default_value='false',
        description='true=启动 Livox + Point-LIO 导航链; false=不启动导航')

    declare_navigation_only = DeclareLaunchArgument(
        'navigation_only', default_value='false',
        description='true=仅启动导航链路(Livox + Point-LIO + 可选全局重定位); false=正常按整车模式启动')

    declare_enable_global_localization = DeclareLaunchArgument(
        'enable_global_localization', default_value='false',
        description='true=启动 small_gicp 全局重定位节点; false=不启动全局重定位')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock for LiDAR / LIO stack')

    declare_lidar_x = DeclareLaunchArgument(
        'lidar_x', default_value='0.0',
        description='Static TF: livox_frame forward offset from base_link (m)')
    declare_lidar_y = DeclareLaunchArgument(
        'lidar_y', default_value='0.2',
        description='Static TF: livox_frame left offset from base_link (m)')
    declare_lidar_z = DeclareLaunchArgument(
        'lidar_z', default_value='0.35',
        description='Static TF: livox_frame height above base_link (m)')
    declare_lidar_roll = DeclareLaunchArgument(
        'lidar_roll', default_value='0.0',
        description='Static TF: livox_frame roll (rad) in base_link frame')
    declare_lidar_pitch = DeclareLaunchArgument(
        'lidar_pitch', default_value='0.3115',
        description='Static TF: livox_frame pitch (rad) in base_link frame — rear raised ~17.85 deg')
    declare_lidar_yaw = DeclareLaunchArgument(
        'lidar_yaw', default_value='1.5708',
        description='Static TF: livox_frame yaw (rad) in base_link frame — front rotated 90 deg left')

    declare_global_map_path = DeclareLaunchArgument(
        'global_map_path', default_value='/home/rm/Desktop/SENTRY_FULL/RMUC2026.pcd',
        description='Absolute path to the static PCD map used by rm_global_localization')

    declare_initial_pose_publish_on_startup = DeclareLaunchArgument(
        'initial_pose_publish_on_startup', default_value='true',
        description='true=启动导航/定位链后自动发一次 /initialpose; false=只等 game_progress 触发')
    declare_enable_nav2 = DeclareLaunchArgument(
        'enable_nav2', default_value='false',
        description='true=在定位链后继续启动纯 Nav2 框架; false=仅启动定位链')
    declare_nav2_map_yaml = DeclareLaunchArgument(
        'nav2_map_yaml', default_value='',
        description='Absolute path to Nav2 static map yaml (required when enable_nav2=true)')
    declare_nav2_params_file = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(bringup_dir, 'config', 'sentry_nav2_params.yaml'),
        description='Nav2 params file used by pure_navigation_bringup.launch.py')
    declare_nav2_rviz = DeclareLaunchArgument(
        'nav2_rviz', default_value='true',
        description='true=Nav2 启动时同时打开 RViz 调试界面')

    use_serial = LaunchConfiguration('use_serial')
    enable_decision = LaunchConfiguration('enable_decision')
    enable_navigation = LaunchConfiguration('enable_navigation')
    navigation_only = LaunchConfiguration('navigation_only')
    enable_global_localization = LaunchConfiguration('enable_global_localization')
    enable_nav2 = LaunchConfiguration('enable_nav2')
    # hw_bridge 只要 use_serial=true 就启动，不受 navigation_only 限制
    serial_hw_enabled = IfCondition(use_serial)
    # 相机/视觉/自瞄需要 use_serial=true 且不是 navigation_only 模式
    serial_enabled = IfCondition(PythonExpression([
        "'", use_serial, "' == 'true' and '", navigation_only, "' == 'false'"
    ]))
    serial_disabled = IfCondition(PythonExpression([
        "'", use_serial, "' == 'false' and '", navigation_only, "' == 'false'"
    ]))
    navigation_enabled = IfCondition(PythonExpression([
        "'", enable_navigation, "' == 'true' or '", navigation_only, "' == 'true'"
    ]))
    navigation_localization_enabled = IfCondition(PythonExpression([
        "('", enable_navigation, "' == 'true' or '", navigation_only, "' == 'true') and '",
        enable_global_localization, "' == 'true'"
    ]))
    navigation_nav2_enabled = IfCondition(PythonExpression([
        "('", enable_navigation, "' == 'true' or '", navigation_only, "' == 'true') and '",
        enable_nav2, "' == 'true'"
    ]))
    decision_enabled = IfCondition(PythonExpression([
        "'", enable_decision, "' == 'true' and '", navigation_only, "' == 'false'"
    ]))

    # ------------------------------------------------------------------
    # 节点 1: rm_hw_bridge — 仅在 use_serial=true 时启动
    # ------------------------------------------------------------------
    hw_bridge_node = Node(
        package='rm_hw_bridge',
        executable='hw_bridge_node',
        name='rm_hw_bridge',
        output='screen',
        parameters=[
            hw_bridge_params,
            {
                'serial_device': LaunchConfiguration('serial_device'),
                'baudrate':      LaunchConfiguration('baudrate'),
            },
        ],
        condition=serial_hw_enabled,  # 只要 use_serial=true 就启动，不受 navigation_only 限制
    )

    # ------------------------------------------------------------------
    # 节点 2a: hik_camera_node — use_serial=true 时延迟 2s 启动
    # ------------------------------------------------------------------
    hik_camera_delayed = TimerAction(
        period=2.0,
        actions=[Node(
            package='rm_hik_driver',
            executable='hik_camera_node',
            name='rm_hik_driver',
            output='screen',
            parameters=[hik_params],
        )],
        condition=serial_enabled,
    )

    # ------------------------------------------------------------------
    # 节点 2b: hik_camera_node — use_serial=false 时立即启动
    # ------------------------------------------------------------------
    hik_camera_immediate = Node(
        package='rm_hik_driver',
        executable='hik_camera_node',
        name='rm_hik_driver',
        output='screen',
        parameters=[hik_params],
        condition=serial_disabled,
    )

    # ------------------------------------------------------------------
    # 节点 2c: 唯一静态 TF — base_link -> livox_frame
    # 统一在主 launch 发布，避免驱动层 / LIO 层重复发布
    # ------------------------------------------------------------------
    livox_static_tf = TimerAction(
        period=0.5,
        actions=[Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_livox_frame',
            arguments=[
                '--x', LaunchConfiguration('lidar_x'),
                '--y', LaunchConfiguration('lidar_y'),
                '--z', LaunchConfiguration('lidar_z'),
                '--roll', LaunchConfiguration('lidar_roll'),
                '--pitch', LaunchConfiguration('lidar_pitch'),
                '--yaw', LaunchConfiguration('lidar_yaw'),
                '--frame-id', 'base_link',
                '--child-frame-id', 'livox_frame',
            ],
        )],
        condition=navigation_enabled,
    )

    # ------------------------------------------------------------------
    # 节点 2d: Livox MID-360 驱动 — 延迟 0.5s 启动
    # ------------------------------------------------------------------
    livox_driver_launch = TimerAction(
        period=0.5,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(livox_launch),
            launch_arguments={
                'rviz': 'false',
                'xfer_format': '4',
                'publish_freq': '20.0',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        )],
        condition=navigation_enabled,
    )

    # ------------------------------------------------------------------
    # 节点 2e: Point-LIO — 在雷达之后启动
    # ------------------------------------------------------------------
    point_lio_delayed = TimerAction(
        period=1.5,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(point_lio_launch),
            launch_arguments={
                'rviz': 'false',
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        )],
        condition=navigation_enabled,
    )

    # ------------------------------------------------------------------
    # 节点 2f: rm_global_localization — 在 Point-LIO 之后启动
    # ------------------------------------------------------------------
    global_localization_delayed = TimerAction(
        period=3.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(global_loc_launch),
            launch_arguments={
                'map_path': LaunchConfiguration('global_map_path'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        )],
        condition=navigation_localization_enabled,
    )

    initial_pose_manager_delayed = TimerAction(
        period=3.5,
        actions=[Node(
            package='rm_bringup',
            executable='initial_pose_manager.py',
            name='rm_initial_pose_manager',
            output='screen',
            parameters=[
                initial_pose_params,
                {
                    'publish_on_startup': LaunchConfiguration('initial_pose_publish_on_startup'),
                },
            ],
        )],
        condition=navigation_localization_enabled,
    )

    nav2_bringup_delayed = TimerAction(
        period=4.5,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pure_nav_launch),
            launch_arguments={
                'map': LaunchConfiguration('nav2_map_yaml'),
                'params_file': LaunchConfiguration('nav2_params_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': 'true',
                'log_level': 'info',
                'rviz': LaunchConfiguration('nav2_rviz'),
            }.items(),
        )],
        condition=navigation_nav2_enabled,
    )

    # ------------------------------------------------------------------
    # 节点 3a: vision_detector — use_serial=true 时延迟 4s 启动
    # ------------------------------------------------------------------
    vision_delayed = TimerAction(
        period=4.0,
        actions=[Node(
            package='rm_vision',
            executable='vision_detector_node',
            name='rm_vision_detector',
            output='screen',
            parameters=[
                vision_params,
                {
                    'color_ignore':        LaunchConfiguration('color_ignore'),
                    'target_color':         LaunchConfiguration('target_color'),
                    'publish_debug_image':  LaunchConfiguration('publish_debug_image'),
                    'model_path':           LaunchConfiguration('model_path'),
                },
            ],
        )],
        condition=serial_enabled,
    )

    # ------------------------------------------------------------------
    # 节点 3b: vision_detector — use_serial=false 时延迟 2s 启动
    # ------------------------------------------------------------------
    vision_quick = TimerAction(
        period=2.0,
        actions=[Node(
            package='rm_vision',
            executable='vision_detector_node',
            name='rm_vision_detector',
            output='screen',
            parameters=[
                vision_params,
                {
                    'color_ignore':        LaunchConfiguration('color_ignore'),
                    'target_color':         LaunchConfiguration('target_color'),
                    'publish_debug_image':  LaunchConfiguration('publish_debug_image'),
                    'model_path':           LaunchConfiguration('model_path'),
                },
            ],
        )],
        condition=serial_disabled,
    )

    # ------------------------------------------------------------------
    # 节点 4a: rm_autoaim — use_serial=true 时延迟 4s 启动 (与 vision 同步)
    # ------------------------------------------------------------------
    autoaim_delayed = TimerAction(
        period=4.0,
        actions=[Node(
            package='rm_autoaim',
            executable='autoaim_node',
            name='rm_autoaim',
            output='screen',
            parameters=[autoaim_params],
        )],
        condition=serial_enabled,
    )

    # ------------------------------------------------------------------
    # 节点 4b: rm_autoaim — use_serial=false 时延迟 2s 启动
    # ------------------------------------------------------------------
    autoaim_quick = TimerAction(
        period=2.0,
        actions=[Node(
            package='rm_autoaim',
            executable='autoaim_node',
            name='rm_autoaim',
            output='screen',
            parameters=[autoaim_params],
        )],
        condition=serial_disabled,
    )

    # ------------------------------------------------------------------
    # 节点 5a: sentry_bt — 默认关闭, 需要时显式 enable_decision:=true
    # use_serial=true 时延迟 5s 启动, 等 /gimbal_status 基本稳定
    # ------------------------------------------------------------------
    decision_delayed = TimerAction(
        period=5.0,
        actions=[Node(
            package='sentry_bt',
            executable='sentry_bt',
            name='sentry_bt',
            output='screen',
        )],
        condition=decision_enabled,
    )

    # ------------------------------------------------------------------
    return LaunchDescription([
        # 参数声明
        declare_use_serial,
        declare_serial_device,
        declare_baudrate,
        declare_color_ignore,
        declare_target_color,
        declare_publish_debug,
        declare_model_path,
        declare_enable_decision,
        declare_enable_navigation,
        declare_navigation_only,
        declare_enable_global_localization,
        declare_use_sim_time,
        declare_lidar_x,
        declare_lidar_y,
        declare_lidar_z,
        declare_lidar_roll,
        declare_lidar_pitch,
        declare_lidar_yaw,
        declare_global_map_path,
        declare_initial_pose_publish_on_startup,
        declare_enable_nav2,
        declare_nav2_map_yaml,
        declare_nav2_params_file,
        declare_nav2_rviz,
        # 节点
        hw_bridge_node,
        hik_camera_delayed,
        hik_camera_immediate,
        livox_static_tf,
        livox_driver_launch,
        point_lio_delayed,
        global_localization_delayed,
        initial_pose_manager_delayed,
        nav2_bringup_delayed,
        vision_delayed,
        vision_quick,
        autoaim_delayed,
        autoaim_quick,
        decision_delayed,
    ])
