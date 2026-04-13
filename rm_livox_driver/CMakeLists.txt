cmake_minimum_required(VERSION 3.10)
project(rm_livox_driver)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files (JSON + RViz)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()