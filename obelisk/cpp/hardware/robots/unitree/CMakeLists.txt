cmake_minimum_required(VERSION 3.16)
project(unitree_interface LANGUAGES CXX)

message(STATUS "Configuring Unitree Interface")

find_package(ament_cmake REQUIRED)

include(FetchContent)
FetchContent_Declare(
    unitree_sdk
    GIT_REPOSITORY https://github.com/unitreerobotics/unitree_sdk2.git
    GIT_TAG 2.0.2)      # Pinned to release v2.0.2
 
set(UNITREE_SDK_BUILD_EXAMPLES OFF CACHE BOOL "Disable building examples")
set(BUILD_EXAMPLES OFF CACHE BOOL "Disable building examples")

FetchContent_MakeAvailable(unitree_sdk)

add_library(UnitreeInterface INTERFACE)
target_include_directories(UnitreeInterface INTERFACE . ${CMAKE_CURRENT_SOURCE_DIR}/../../include)

target_link_libraries(UnitreeInterface INTERFACE Obelisk::Core unitree_sdk2)

ament_target_dependencies(UnitreeInterface INTERFACE
  rclcpp
  rclcpp_lifecycle
  sensor_msgs)


# add_executable(unitree_test unitree_test.cpp)
# target_link_libraries(unitree_test unitree_sdk2)

# TODO:
# - Run without Pixi
# - Install Eigen and Ping