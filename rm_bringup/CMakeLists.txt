cmake_minimum_required(VERSION 3.10)
project(rm_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

# ---- obstacle_cloud_filter_node (Nav2 障碍点云过滤) ----
add_executable(obstacle_cloud_filter_node
  src/obstacle_cloud_filter_node.cpp
)

target_compile_features(obstacle_cloud_filter_node PUBLIC cxx_std_17)

target_include_directories(obstacle_cloud_filter_node PRIVATE
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(obstacle_cloud_filter_node
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

ament_target_dependencies(obstacle_cloud_filter_node
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2_ros
)

# ---- self_point_filter_node (建图链自车点云剔除) ----
add_executable(self_point_filter_node
  src/self_point_filter_node.cpp
)

target_compile_features(self_point_filter_node PUBLIC cxx_std_17)

target_link_libraries(self_point_filter_node
  Eigen3::Eigen
)

ament_target_dependencies(self_point_filter_node
  livox_ros_driver2
  rclcpp
  sensor_msgs
  tf2_ros
)

add_executable(localization_recovery_manager_node
  src/localization_recovery_manager_node.cpp
)

target_compile_features(localization_recovery_manager_node PUBLIC cxx_std_17)

target_link_libraries(localization_recovery_manager_node
  Eigen3::Eigen
)

ament_target_dependencies(localization_recovery_manager_node
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  std_srvs
  tf2_ros
)

install(TARGETS obstacle_cloud_filter_node self_point_filter_node localization_recovery_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安装配置文件
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 安装 RViz 配置
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# 安装轻量脚本节点
install(PROGRAMS
  scripts/cmd_vel_to_nav_cmd.py
  scripts/pb_cmd_vel_to_nav_cmd.py
  scripts/initial_pose_manager.py
  scripts/pcd2pgm.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
