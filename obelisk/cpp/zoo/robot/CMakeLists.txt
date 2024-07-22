cmake_minimum_required(VERSION 3.16)
project(dynamixel_interface LANGUAGES CXX)

message(STATUS "Configuring Dynamixel Interface")

# Find the Dynamixel SDK library
find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)

include_directories("${dynamixel_sdk_INCLUDE_DIRS}/dynamixel_sdk")

add_executable(dynamixel_control leap_hand_interface.h)

ament_target_dependencies(dynamixel_control
  dynamixel_sdk
)
