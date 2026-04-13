# =============================================================================
# hik_camera.launch.py — 海康相机驱动节点启动文件
# =============================================================================
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('rm_hik_driver')
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='rm_hik_driver',
            executable='hik_camera_node',
            name='rm_hik_driver',
            output='screen',
            parameters=[params_file],
            # ★ 设置 LD_LIBRARY_PATH 以便运行时找到 libMvCameraControl.so
            # 如果已安装到系统路径则不需要
        ),
    ])