cmake_minimum_required(VERSION 3.10)
project(rm_bringup)

find_package(ament_cmake REQUIRED)

# 安装 launch 文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()