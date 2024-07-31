cmake_minimum_required(VERSION 3.16)
project(dynamixel_interface LANGUAGES CXX)

message(STATUS "Configuring Dynamixel Interface")

# Find the Dynamixel SDK library
find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)

add_library(LeapHandInterface INTERFACE)
target_include_directories(LeapHandInterface INTERFACE .)
target_link_libraries(LeapHandInterface INTERFACE dynamixel_sdk)
