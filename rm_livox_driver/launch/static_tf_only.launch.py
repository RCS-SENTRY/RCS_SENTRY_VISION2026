"""
rm_livox_driver / static_tf_only.launch.py
===========================================
Launch ONLY the static TF publisher: base_link -> livox_frame.

Use this for:
  - Verifying TF tree correctness in isolation
  - Debugging TF connectivity with `ros2 run tf2_ros tf2_echo base_link livox_frame`
  - Combining with other modules launched separately

Usage:
  ros2 launch rm_livox_driver static_tf_only.launch.py
  ros2 launch rm_livox_driver static_tf_only.launch.py z:=0.3
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    x_arg = DeclareLaunchArgument('x', default_value='0.0')
    y_arg = DeclareLaunchArgument('y', default_value='0.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.5')
    roll_arg = DeclareLaunchArgument('roll', default_value='0.0')
    pitch_arg = DeclareLaunchArgument('pitch', default_value='0.0')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0')

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_livox_frame',
        arguments=[
            '--x',      LaunchConfiguration('x'),
            '--y',      LaunchConfiguration('y'),
            '--z',      LaunchConfiguration('z'),
            '--roll',   LaunchConfiguration('roll'),
            '--pitch',  LaunchConfiguration('pitch'),
            '--yaw',    LaunchConfiguration('yaw'),
            '--frame-id', 'base_link',
            '--child-frame-id', 'livox_frame',
        ],
    )

    return LaunchDescription([
        x_arg, y_arg, z_arg, roll_arg, pitch_arg, yaw_arg,
        static_tf,
    ])