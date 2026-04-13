cmake_minimum_required(VERSION 3.8)
project(rm_vision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

# ---- ROS 2 依赖 ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rm_interfaces REQUIRED)

# ---- OpenCV + OpenVINO ----
find_package(OpenCV REQUIRED)

set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake/")
find_package(OpenVINO REQUIRED)

# ---- 可执行目标 ----
add_executable(vision_detector_node
  src/vision_detector_node.cpp
  src/yolo_detector.cpp
)

target_include_directories(vision_detector_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(vision_detector_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  rm_interfaces
)

target_link_libraries(vision_detector_node
  ${OpenCV_LIBS}
  openvino::runtime
)

# ---- 安装 ----
install(TARGETS vision_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()