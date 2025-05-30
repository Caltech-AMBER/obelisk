cmake_minimum_required(VERSION 3.16)

project(obelisk_control_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PositionSetpoint.msg"
  "msg/PDFeedForward.msg"
  "msg/ExecutionFSM.msg"
  "msg/VelocityCommand.msg"
  DEPENDENCIES
  std_msgs
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  std_msgs
  rosidl_default_generators
)
include_directories(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
