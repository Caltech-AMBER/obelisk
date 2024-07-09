message(STATUS "Building Obelisk Cpp")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

# ROS 2 Packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcl REQUIRED)

# Obelisk Messages
find_package(obelisk_control_msgs REQUIRED)
find_package(obelisk_sensor_msgs REQUIRED)
find_package(obelisk_estimator_msgs REQUIRED)

# ------- Mujoco ------- #
include(FetchContent)
set(MUJOCO_VERSION "3.1.6" CACHE STRING "mujoco version")
set(COMP_ARCH "x86_64" CACHE STRING "computer architecture.")

FetchContent_Declare(
  mujoco
  URL https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-${COMP_ARCH}.tar.gz
)

FetchContent_Populate(mujoco)

# Setup the Mujoco library
find_library(MUJOCO_LIB mujoco-lib REQUIRED HINTS ${mujoco_SOURCE_DIR}/lib)
add_library(mujoco-lib SHARED IMPORTED GLOBAL)
target_include_directories(mujoco-lib INTERFACE ${mujoco_SOURCE_DIR}/include)
target_include_directories(mujoco-lib INTERFACE ${mujoco_SOURCE_DIR}/include/mujoco)

set_property(TARGET mujoco-lib PROPERTY IMPORTED_LOCATION ${mujoco_SOURCE_DIR}/lib/libmujoco.so)

# GLFW
find_package(glfw3 REQUIRED)


# Source files
set(OBELISK_CPP_INC "${obelisk_cpp_SOURCE_DIR}/obelisk_cpp/include")
set(OBELISK_CPP_HEADER_LIST
        "${OBELISK_CPP_INC}/obelisk_node.h"
        "${OBELISK_CPP_INC}/obelisk_controller.h"
        "${OBELISK_CPP_INC}/obelisk_estimator.h"
        "${OBELISK_CPP_INC}/obelisk_sensor.h"
        "${OBELISK_CPP_INC}/obelisk_robot.h"
        "${OBELISK_CPP_INC}/obelisk_sim_robot.h"
        "${OBELISK_CPP_INC}/obelisk_mujoco_sim_robot")

add_library(ObeliskCpp INTERFACE)
target_include_directories(ObeliskCpp INTERFACE include ${mujoco_SOURCE_DIR}/include)

target_link_libraries(ObeliskCpp INTERFACE mujoco-lib glfw) #glfw)

ament_target_dependencies(ObeliskCpp INTERFACE
  rclcpp
  rclcpp_lifecycle
  obelisk_control_msgs
  obelisk_estimator_msgs
  obelisk_sensor_msgs
  std_msgs)

target_include_directories(ObeliskCpp INTERFACE
    $<BUILD_INTERFACE:${OBELISK_CPP_INC}>
    $<INSTALL_INTERFACE:$obelisk_cpp/include>)
