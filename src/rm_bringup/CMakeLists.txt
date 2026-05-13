cmake_minimum_required(VERSION 3.10)
project(rm_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(second_lidar_safety_limiter_node
  src/second_lidar_safety_limiter_node.cpp
)
ament_target_dependencies(second_lidar_safety_limiter_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(PROGRAMS
  scripts/livox_custom_to_pointcloud2.py
  scripts/pb_cmd_vel_to_nav_cmd.py
  scripts/sentry_match_nav_sim.py
  scripts/second_lidar_obstacle_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  second_lidar_safety_limiter_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
