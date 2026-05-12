cmake_minimum_required(VERSION 3.16)
project(westwood_interface LANGUAGES CXX)

message(STATUS "Configuring Westwood Interface")

# No vendor SDK to fetch — the wire protocol is hand-rolled UDP, and the on-robot
# AOS bridge (server/themis_obk_server.py) is the only Python piece.

find_package(ament_cmake REQUIRED)

add_library(WestwoodInterface INTERFACE)
add_library(Westwood::Interface ALIAS WestwoodInterface)

target_include_directories(WestwoodInterface INTERFACE
    .                                      # themis_interface.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include  # themis_fsm.h, themis_example_controller.h
)

target_link_libraries(WestwoodInterface INTERFACE Obelisk::Core)

ament_target_dependencies(WestwoodInterface INTERFACE
    rclcpp
    rclcpp_lifecycle
    sensor_msgs
)
