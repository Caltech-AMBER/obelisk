cmake_minimum_required(VERSION 3.16)
project(dynamixel_interface LANGUAGES CXX)

message(STATUS "Configuring Dynamixel Interface")

# Find the Dynamixel SDK library
find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)

add_library(dynamixel_control INTERFACE)
target_include_directories(dynamixel_control INTERFACE .)
target_link_libraries(dynamixel_control INTERFACE dynamixel_sdk)
