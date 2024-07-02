message(STATUS "Building Obelisk Cpp")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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
# Setup Mujoco path
set(MUJOCO_VERSION "mujoco-3.1.6" CACHE STRING "mujoco version")
set(MUJOCO_DIR "/home/$ENV{USER}/mujoco/${MUJOCO_VERSION}" CACHE PATH "Path to Mujoco dir")
message(STATUS "mujoco directory: ${MUJOCO_DIR}")

# Setup the Mujoco library
find_library(MUJOCO_LIB mujoco REQUIRED HINTS ${MUJOCO_DIR}/lib)
add_library(mujoco SHARED IMPORTED GLOBAL)
target_include_directories(mujoco INTERFACE ${MUJOCO_DIR}/include)
target_include_directories(mujoco INTERFACE ${MUJOCO_DIR}/include/mujoco)

set_property(TARGET mujoco PROPERTY IMPORTED_LOCATION ${MUJOCO_DIR}/lib/libmujoco.so)


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

add_library(ObeliskCpp ${OBELISK_CPP_HEADER_LIST})
set_target_properties(ObeliskCpp PROPERTIES LINKER_LANGUAGE CXX)

target_link_libraries(ObeliskCpp PUBLIC mujoco)

ament_target_dependencies(ObeliskCpp PUBLIC
  rclcpp
  rclcpp_lifecycle
  obelisk_control_msgs
  obelisk_estimator_msgs
  obelisk_sensor_msgs
  std_msgs)

target_include_directories(ObeliskCpp PUBLIC
    $<BUILD_INTERFACE:${OBELISK_CPP_INC}>
    $<INSTALL_INTERFACE:$obelisk_cpp/include>)
