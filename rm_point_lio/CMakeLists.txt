# ============================================================================
# rm_point_lio — Pure config/launch wrapper, NO C++ code
# ============================================================================
cmake_minimum_required(VERSION 3.10)
project(rm_point_lio)

find_package(ament_cmake REQUIRED)

# Install config and launch files
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()