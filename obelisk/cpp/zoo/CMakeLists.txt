message(STATUS "Configuring Obelisk Zoo")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------- ROS 2 Packages ------- #
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcl REQUIRED)

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
message(STATUS "OBELISK_BUILD_LEAP cmake variable: ${OBELISK_BUILD_LEAP}")
message(STATUS "OBELISK_BUILD_LEAP env variable: $ENV{OBELISK_BUILD_LEAP}")
message(STATUS "If either variable is true, the Leap Hand interface will be built.")

if(((DEFINED OBELISK_BUILD_LEAP) AND (OBELISK_BUILD_LEAP STREQUAL "true")) OR
    (DEFINED ENV{OBELISK_BUILD_LEAP} AND $ENV{OBELISK_BUILD_LEAP} STREQUAL "true"))
  message(STATUS "Configuring Leap Hand Interface")
  add_subdirectory(hardware/robots/leap)
  target_include_directories(Zoo INTERFACE hardware/robots/leap)
else()
  message(STATUS "Leap Hand Interface not configured")
endif()

# conditionally add zed subdirectory if OBELISK_BUILD_ZED is set
message(STATUS "OBELISK_BUILD_ZED: ${OBELISK_BUILD_ZED}")
message(STATUS "OBELISK_BUILD_ZED env variable: $ENV{OBELISK_BUILD_ZED}")
message(STATUS "If either variable is true, the Zed interface will be built.")

if((DEFINED OBELISK_BUILD_ZED AND OBELISK_BUILD_ZED STREQUAL "true") OR
   (DEFINED ENV{OBELISK_BUILD_ZED} AND $ENV{OBELISK_BUILD_ZED} STREQUAL "true"))
  message(STATUS "Configuring Zed2 Sensors")
  add_subdirectory(hardware/sensing/zed)
  target_include_directories(Zoo INTERFACE hardware/sensing/zed)
else()
  message(STATUS "Zed2 Sensors not configured")
endif()

target_link_libraries(Zoo INTERFACE Obelisk::Core)

ament_target_dependencies(Zoo INTERFACE
  rclcpp
  rclcpp_lifecycle)
