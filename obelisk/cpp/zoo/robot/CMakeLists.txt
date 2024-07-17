cmake_minimum_required(VERSION 3.5)
message(STATUS "Configuring Dynamixel Interface")

# Find the Dynamixel SDK library
find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp REQUIRED)


# Check if the library was found
if(NOT dynamixel_sdk)
  message(FATAL_ERROR "Dynamixel SDK library not found")
else()
  message(STATUS "Dynamixel SDK library found at ${DYNAMIXEL_SDK_LIB}")
endif()


# add_executable(dynamixel_control leap_test.cpp)
add_executable(dynamixel_control leap_hand_interface.h)

# Include the Dynamixel SDK headers
target_include_directories(dynamixel_control PUBLIC "$ENV{OBELISK_ROOT}/.pixi/envs/solve-group-envs/dev/include/dynamixel_sdk")

ament_target_dependencies(dynamixel_control
  dynamixel_sdk
  rclcpp
  rclcpp_lifecycle
)

# Link the Dynamixel SDK library to your executable
target_link_libraries(dynamixel_control dynamixel_sdk)
