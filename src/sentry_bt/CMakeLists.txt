cmake_minimum_required(VERSION 3.8)
project(sentry_bt)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(TINYXML2 REQUIRED IMPORTED_TARGET tinyxml2)

find_package(behaviortree_cpp QUIET)
if(behaviortree_cpp_FOUND)
  set(BT_CPP_TARGET behaviortree_cpp::behaviortree_cpp)
else()
  find_package(behaviortree_cpp_v3 REQUIRED)
  set(BT_CPP_TARGET BT::behaviortree_cpp_v3)
endif()

include_directories(include)

set(SENTRY_BT_SOURCES
    src/bt_nodes_basic.cpp
    src/bt_nodes_guard.cpp
    src/bt_nodes_tactical.cpp
    src/bt_nodes_rule.cpp
    src/bt_nodes_executor.cpp
    src/bt_setup.cpp
    src/referee_interface.cpp
    src/llm_interface.cpp
)

add_executable(sentry_bt
    main.cpp
    ${SENTRY_BT_SOURCES}
)

add_executable(sentry_bt_export_tree
    src/export_tree_model_main.cpp
    ${SENTRY_BT_SOURCES}
)

add_executable(sentry_bt_sim
    src/sentry_bt_sim_main.cpp
)

add_executable(sentry_bt_scenario_runner
    src/sentry_bt_scenario_runner.cpp
    ${SENTRY_BT_SOURCES}
)

target_compile_definitions(sentry_bt PRIVATE SENTRY_BT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(sentry_bt_export_tree PRIVATE SENTRY_BT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(sentry_bt_sim PRIVATE SENTRY_BT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_compile_definitions(sentry_bt_scenario_runner PRIVATE SENTRY_BT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(sentry_bt ${BT_CPP_TARGET} PkgConfig::TINYXML2)
target_link_libraries(sentry_bt_export_tree ${BT_CPP_TARGET} PkgConfig::TINYXML2)
target_link_libraries(sentry_bt_sim yaml-cpp)
target_link_libraries(sentry_bt_scenario_runner ${BT_CPP_TARGET} PkgConfig::TINYXML2 yaml-cpp)
ament_target_dependencies(sentry_bt rclcpp rm_interfaces)
ament_target_dependencies(sentry_bt_export_tree rclcpp rm_interfaces)
ament_target_dependencies(sentry_bt_sim rclcpp rm_interfaces)
ament_target_dependencies(sentry_bt_scenario_runner rclcpp rm_interfaces)

add_custom_target(generate_groot_xml ALL
    COMMAND $<TARGET_FILE:sentry_bt_export_tree>
    DEPENDS sentry_bt_export_tree ${CMAKE_CURRENT_SOURCE_DIR}/tree/sentry_main.template.xml
    COMMENT "Generating BehaviorTree XML with TreeNodesModel for Groot2"
    VERBATIM
)

add_dependencies(sentry_bt generate_groot_xml)

install(TARGETS
    sentry_bt
    sentry_bt_export_tree
    sentry_bt_sim
    sentry_bt_scenario_runner
    DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
    scripts/sentry_bt_debug_watch
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    include/
    launch/
    tree/
    scenarios/
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
