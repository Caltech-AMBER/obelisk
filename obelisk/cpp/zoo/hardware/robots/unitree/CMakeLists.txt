cmake_minimum_required(VERSION 3.16)
project(unitree_interface LANGUAGES CXX)

message(STATUS "Configuring Unitree Interface")

find_package(ament_cmake REQUIRED)

include(FetchContent)
FetchContent_Declare(
    unitree_sdk
    GIT_REPOSITORY https://github.com/unitreerobotics/unitree_sdk2.git
    GIT_TAG 3a4680ae9b00df59e60f7e63cfb0fcc432a9d08d) # main # TODO: Fix the issue here

set(UNITREE_SDK_BUILD_EXAMPLES OFF CACHE BOOL "Disable building examples")
set(BUILD_EXAMPLES OFF CACHE BOOL "Disable building examples")

FetchContent_MakeAvailable(unitree_sdk)

add_library(UnitreeInterface INTERFACE)
target_include_directories(UnitreeInterface INTERFACE .)

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
