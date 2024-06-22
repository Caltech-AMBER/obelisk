cmake_minimum_required(VERSION 3.5)

project(obelisk_estimator_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EstimatedState.msg"
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
