cmake_minimum_required(VERSION 3.10)
project(rm_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(PROGRAMS
  scripts/livox_custom_to_pointcloud2.py
  scripts/pb_cmd_vel_to_nav_cmd.py
  scripts/second_lidar_obstacle_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
