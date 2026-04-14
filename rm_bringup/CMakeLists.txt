cmake_minimum_required(VERSION 3.10)
project(rm_bringup)

find_package(ament_cmake REQUIRED)

# 安装 launch 文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安装配置文件
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 安装轻量脚本节点
install(PROGRAMS
  scripts/cmd_vel_to_nav_cmd.py
  scripts/initial_pose_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
