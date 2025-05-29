cmake_minimum_required(VERSION 3.16)

project(obelisk_sensor_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(obelisk_std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObkJointEncoders.msg"
  "msg/TrueSimState.msg"
  "msg/ObkImage.msg"
  "msg/ObkImu.msg"
  "msg/ObkFramePose.msg"
  "msg/ObkForceSensor.msg"
  "msg/ObkScanDots.msg"
  DEPENDENCIES
  std_msgs
  geometry_msgs
  obelisk_std_msgs
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  std_msgs
  obelisk_std_msgs
  rosidl_default_generators
)
include_directories(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
