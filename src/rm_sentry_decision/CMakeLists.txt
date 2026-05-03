cmake_minimum_required(VERSION 3.8)
project(rm_sentry_decision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(sentry_command_mux_node
  src/sentry_command_mux_node.cpp
)
ament_target_dependencies(sentry_command_mux_node
  rclcpp
  rm_interfaces
  std_msgs
)

add_executable(sentry_goal_executor_node
  src/sentry_goal_executor_node.cpp
)
target_link_libraries(sentry_goal_executor_node yaml-cpp)
ament_target_dependencies(sentry_goal_executor_node
  geometry_msgs
  nav2_msgs
  rclcpp
  rclcpp_action
  rm_interfaces
  std_msgs
)

install(TARGETS
  sentry_command_mux_node
  sentry_goal_executor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
