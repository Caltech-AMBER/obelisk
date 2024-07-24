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
target_include_directories(Zoo INTERFACE robot)

target_link_libraries(Zoo INTERFACE Obelisk::Core)

ament_target_dependencies(Zoo INTERFACE
  rclcpp
  rclcpp_lifecycle)
