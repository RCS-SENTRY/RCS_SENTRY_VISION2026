cmake_minimum_required(VERSION 3.10)
project(rm_hik_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wpedantic)

# ---- 依赖 ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)

# ---- 海康 MVS SDK ----
set(MVS_SDK_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include/MvCameraControl")
set(MVS_SDK_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib/amd64")

# ---- 核心节点 ----
add_executable(hik_camera_node
  src/hik_camera_node.cpp
)

target_include_directories(hik_camera_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${MVS_SDK_INCLUDE_DIR}
)

target_link_libraries(hik_camera_node
  ${MVS_SDK_LIB_DIR}/libMvCameraControl.so
)

ament_target_dependencies(hik_camera_node
  rclcpp
  sensor_msgs
  camera_info_manager
)

# ---- 安装 ----
install(TARGETS hik_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY lib/
  DESTINATION share/${PROJECT_NAME}/lib
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()