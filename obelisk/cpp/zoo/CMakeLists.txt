message(STATUS "Configuring Obelisk Zoo")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------- ROS 2 Packages ------- #
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcl REQUIRED)
find_package(sensor_msgs REQUIRED)

set(ZOO_INC "${obelisk_cpp_SOURCE_DIR}/obelisk_cpp/include")

# Not currently used, but might be used in the future
# set(ZOO_HEADER_LIST
#         "${OBELISK_CPP_INC}/jointencoders_passthrough_estimator.h"
#         "${OBELISK_CPP_INC}/position_setpoint_controller.h")

# ------- Making the library ------- #
add_library(Zoo INTERFACE)
add_library(Obelisk::Zoo ALIAS Zoo)

target_include_directories(Zoo INTERFACE include)

# conditionally add leap subdirectory if OBELISK_BUILD_LEAP is set
# Check if both conditions are met using if
if(DEFINED OBELISK_BUILD_LEAP AND OBELISK_BUILD_LEAP STREQUAL "true")
    set(LEAP_VAR_CMAKE TRUE)
else()
    set(LEAP_VAR_CMAKE FALSE)
endif()

# Check the environment variable using if
if(DEFINED ENV{OBELISK_BUILD_LEAP} AND "$ENV{OBELISK_BUILD_LEAP}" STREQUAL "true")
    set(LEAP_VAR_ENV TRUE)
else()
    set(LEAP_VAR_ENV FALSE)
endif()

message(STATUS "LEAP_VAR_CMAKE: ${LEAP_VAR_CMAKE}")
message(STATUS "LEAP_VAR_ENV env variable: ${LEAP_VAR_ENV}")
message(STATUS "If either variable is true, the Zed interface will be built.")

if(LEAP_VAR_CMAKE OR LEAP_VAR_ENV)
  message(STATUS "Configuring Leap Hand Interface")
  add_subdirectory(hardware/robots/leap)
  target_include_directories(Zoo INTERFACE hardware/robots/leap)
else()
  message(STATUS "Leap Hand Interface not configured")
endif()

# conditionally add zed subdirectory if OBELISK_BUILD_ZED is set
# Check if both conditions are met using if
if(DEFINED OBELISK_BUILD_ZED AND OBELISK_BUILD_ZED STREQUAL "true")
    set(ZED_VAR_CMAKE TRUE)
else()
    set(ZED_VAR_CMAKE FALSE)
endif()

# Check the environment variable using if
if(DEFINED ENV{OBELISK_BUILD_ZED} AND "$ENV{OBELISK_BUILD_ZED}" STREQUAL "true")
    set(ZED_VAR_ENV TRUE)
else()
    set(ZED_VAR_ENV FALSE)
endif()

message(STATUS "ZED_VAR_CMAKE: ${ZED_VAR_CMAKE}")
message(STATUS "ZED_VAR_ENV env variable: ${ZED_VAR_ENV}")
message(STATUS "If either variable is true, the Zed interface will be built.")

if(ZED_VAR_CMAKE OR ZED_VAR_ENV)
  message(STATUS "Configuring Zed2 Sensors")
  add_subdirectory(hardware/sensing/zed)
  target_include_directories(Zoo INTERFACE hardware/sensing/zed)
else()
  message(STATUS "Zed2 Sensors not configured")
endif()

# Check the environment variable using if
message(STATUS "OBELISK_BUILD_UNITREE env variable: ${OBELISK_BUILD_UNITREE}")
message(STATUS $ENV{OBELISK_BUILD_UNITREE})
if(DEFINED ENV{OBELISK_BUILD_UNITREE} AND "$ENV{OBELISK_BUILD_UNITREE}" STREQUAL "true")
    set(UNITREE_VAR_ENV TRUE)
else()
    set(UNITREE_VAR_ENV FALSE)
endif()

if(UNITREE_VAR_ENV)
  message(STATUS "Configuring Unitree interface...")
  add_subdirectory(hardware/robots/unitree)
endif()

target_link_libraries(Zoo INTERFACE Obelisk::Core)

ament_target_dependencies(Zoo INTERFACE
  rclcpp
  rclcpp_lifecycle
  sensor_msgs)
