message(STATUS "Configuring Obelisk Cpp")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------- ROS 2 Packages ------- #
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcl REQUIRED)

# ------- Obelisk Messages ------- #
find_package(obelisk_control_msgs REQUIRED)
find_package(obelisk_sensor_msgs REQUIRED)
find_package(obelisk_estimator_msgs REQUIRED)
find_package(obelisk_std_msgs REQUIRED)

# ------- Mujoco ------- #
include(FetchContent)
set(MUJOCO_VERSION "3.2.0" CACHE STRING "mujoco version")
set(COMP_ARCH "x86_64" CACHE STRING "computer architecture")

FetchContent_Declare(
  mujoco
  URL https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-${COMP_ARCH}.tar.gz
)

FetchContent_MakeAvailable(mujoco)

# ------- Setup the Mujoco library ------- #
message(STATUS "mujoco source dir: ${mujoco_SOURCE_DIR}")
find_library(MUJOCO_LIB mujoco REQUIRED HINTS ${mujoco_SOURCE_DIR}/lib)
add_library(mujoco-lib SHARED IMPORTED GLOBAL)
target_include_directories(mujoco-lib INTERFACE ${mujoco_SOURCE_DIR}/include)
target_include_directories(mujoco-lib INTERFACE ${mujoco_SOURCE_DIR}/include/mujoco)

set_property(TARGET mujoco-lib PROPERTY IMPORTED_LOCATION ${mujoco_SOURCE_DIR}/lib/libmujoco.so)

# ------- GLFW ------- #
find_package(glfw3 REQUIRED)

# ------- Source files ------- #
message(STATUS "cmake source dir: ${CMAKE_CURRENT_SOURCE_DIR}")
set(CORE_INC "${CMAKE_CURRENT_SOURCE_DIR}/include")

# These are not currently used, but might be used in the future
# set(OBELISK_CPP_HEADER_LIST
#         "${CORE_INC}/obelisk_node.h"
#         "${CORE_INC}/obelisk_controller.h"
#         "${CORE_INC}/obelisk_estimator.h"
#         "${CORE_INC}/obelisk_sensor.h"
#         "${CORE_INC}/obelisk_robot.h"
#         "${CORE_INC}/obelisk_sim_robot.h"
#         "${CORE_INC}/obelisk_mujoco_sim_robot")

# ------- Making the library ------- #
add_library(ObkCore INTERFACE)
add_library(Obelisk::Core ALIAS ObkCore)   # Namespaced alias
target_include_directories(ObkCore INTERFACE ${CORE_INC} ${mujoco_SOURCE_DIR}/include)

target_link_libraries(ObkCore INTERFACE mujoco-lib glfw)

ament_target_dependencies(ObkCore INTERFACE
  rclcpp
  rclcpp_lifecycle
  obelisk_control_msgs
  obelisk_estimator_msgs
  obelisk_sensor_msgs
  std_msgs)
