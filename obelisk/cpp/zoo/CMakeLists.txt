message(STATUS "Building Obelisk Zoo")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 Packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcl REQUIRED)

set(OBELISK_ZOO_INC "${obelisk_cpp_SOURCE_DIR}/obelisk_cpp/include")
set(OBELISK_ZOO_HEADER_LIST
        "${OBELISK_CPP_INC}/jointencoders_passthrough_estimator.h"
        "${OBELISK_CPP_INC}/position_setpoint_controller.h")

add_library(Zoo INTERFACE)

target_include_directories(Zoo INTERFACE include)

target_link_libraries(Zoo INTERFACE ObeliskCpp)

ament_target_dependencies(Zoo INTERFACE
  rclcpp
  rclcpp_lifecycle)

target_include_directories(Zoo INTERFACE
    $<BUILD_INTERFACE:${OBELISK_ZOO_INC}>
    $<INSTALL_INTERFACE:$zoo/include>)
