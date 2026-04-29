cmake_minimum_required(VERSION 3.11)
project(small_gicp_relocalization)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
# add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
find_package(OpenMP REQUIRED)
ament_auto_find_build_dependencies()

# Prefer the workspace-vendored small_gicp package. Fall back to the upstream
# FetchContent path when this package is built outside the XMU workspace.
find_package(small_gicp QUIET)
if(small_gicp_FOUND)
  set(SMALL_GICP_TARGET small_gicp::small_gicp)
else()
  include(FetchContent)
  fetchcontent_declare(
    small_gicp
    GIT_REPOSITORY https://github.com/koide3/small_gicp.git
    GIT_TAG "master"
  )
  fetchcontent_makeavailable(small_gicp)
  set(SMALL_GICP_TARGET small_gicp)
endif()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX ${SMALL_GICP_TARGET})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN small_gicp_relocalization::SmallGicpRelocalizationNode
  EXECUTABLE ${PROJECT_NAME}_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
  set(ament_cmake_clang_tidy_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-tidy")
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
)
