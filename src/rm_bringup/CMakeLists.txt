cmake_minimum_required(VERSION 3.10)
project(rm_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(PROGRAMS
  scripts/pb_cmd_vel_to_nav_cmd.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
