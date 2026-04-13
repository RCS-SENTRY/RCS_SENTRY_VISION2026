cmake_minimum_required(VERSION 3.8)
project(rm_global_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(small_gicp REQUIRED)

add_executable(global_localization_node
  src/global_localization_node.cpp
)

target_compile_features(global_localization_node PUBLIC cxx_std_17)

target_include_directories(global_localization_node PRIVATE
  include
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(global_localization_node
  Eigen3::Eigen
  small_gicp::small_gicp
  ${PCL_LIBRARIES}
)

ament_target_dependencies(global_localization_node
  geometry_msgs
  pcl_conversions
  rclcpp
  rm_interfaces
  sensor_msgs
  tf2_ros
)

install(TARGETS global_localization_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
