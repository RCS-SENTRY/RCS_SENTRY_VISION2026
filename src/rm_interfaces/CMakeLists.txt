cmake_minimum_required(VERSION 3.8)
project(rm_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GimbalCmd.msg"
  "msg/GimbalStatus.msg"
  "msg/NavCmd.msg"
  "msg/ArmorDetection.msg"
  "msg/ArmorDetections.msg"
  "msg/LocalizationStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
