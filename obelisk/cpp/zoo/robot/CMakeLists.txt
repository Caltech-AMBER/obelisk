cmake_minimum_required(VERSION 3.16)

project(obk_leap_test)

message(STATUS "Configuring Dynamixel Interface")

# Find the Dynamixel SDK library
find_library(DYNAMIXEL_SDK_LIB dynamixel_sdk)

# Check if the library was found
if(NOT DYNAMIXEL_SDK_LIB)
  message(FATAL_ERROR "Dynamixel SDK library not found")
else()
  message(STATUS "Dynamixel SDK library found at ${DYNAMIXEL_SDK_LIB}")
endif()

# Include the Dynamixel SDK headers
include_directories($ENV{OBELISK_ROOT}/.pixi/envs/solve-group-envs/dev/include/dynamixel_sdk)

# Add an executable (replace main.cpp with your source file)
add_executable(dynamixel_control leap_test.cpp)

# Link the Dynamixel SDK library to your executable
target_link_libraries(dynamixel_control ${DYNAMIXEL_SDK_LIB})
