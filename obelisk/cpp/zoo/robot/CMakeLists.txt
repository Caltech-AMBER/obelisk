cmake_minimum_required(VERSION 3.5)
project(dynamixel_interface LANGUAGES CXX)

message(STATUS "Configuring Dynamixel Interface")

# Find the Dynamixel SDK library
find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)

# add_executable(dynamixel_control leap_test.cpp)
add_executable(dynamixel_control leap_hand_interface.h)

ament_target_dependencies(dynamixel_control
  dynamixel_sdk
)
