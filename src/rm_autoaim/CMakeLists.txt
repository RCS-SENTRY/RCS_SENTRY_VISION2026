cmake_minimum_required(VERSION 3.8)
project(rm_autoaim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

# ---- ROS 2 依赖 ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rm_interfaces REQUIRED)

# ---- OpenCV ----
find_package(OpenCV REQUIRED)

# ---- Eigen3 ----
find_package(Eigen3 REQUIRED)

# ---- 追踪器库 (纯数学，零 ROS 依赖) ----
add_library(imm_ukf_tracker STATIC
  src/imm_ukf_tracker.cpp
)
target_include_directories(imm_ukf_tracker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(imm_ukf_tracker
  Eigen3::Eigen
)

# ---- 自瞄节点 ----
add_executable(autoaim_node
  src/autoaim_node.cpp
  src/core/csu_armor_tracker.cpp
  src/core/manual_compensator.cpp
)

target_include_directories(autoaim_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(autoaim_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  rm_interfaces
)

target_link_libraries(autoaim_node
  ${OpenCV_LIBS}
  imm_ukf_tracker
  Eigen3::Eigen
)

# ---- 安装 ----
install(TARGETS autoaim_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
