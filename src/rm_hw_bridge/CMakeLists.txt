cmake_minimum_required(VERSION 3.8)
project(rm_hw_bridge)

# ============================================================================
# 编译选项
# ============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wpedantic)

# ============================================================================
# 依赖
# ============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rm_interfaces REQUIRED)

# ============================================================================
# 头文件路径
# ============================================================================
include_directories(include)

# ============================================================================
# 可执行目标
# ============================================================================
add_executable(hw_bridge_node
  src/hw_bridge_node.cpp
)

ament_target_dependencies(hw_bridge_node
  rclcpp
  sensor_msgs
  std_msgs
  rm_interfaces
)

# 链接 rt 和 pthread（POSIX 串口 / select）
target_link_libraries(hw_bridge_node rt pthread)

# ============================================================================
# 安装
# ============================================================================
install(TARGETS hw_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ============================================================================
# 测试（占位）
# ============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()