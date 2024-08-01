cmake_minimum_required(VERSION 3.16)
project(obelisk_zed_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Get the Obelisk library
set(OBELISK_BUILD_ZED ON)
include(FetchContent)
FetchContent_Declare(
  Obelisk
  SOURCE_DIR $ENV{OBELISK_ROOT}/obelisk/cpp
)

# Lets us Get Obelisk::Core and Obelisk::Zoo
FetchContent_MakeAvailable(Obelisk)

# zed2_sensors
add_executable(zed2_sensors src/zed2_sensors.cpp)
target_link_libraries(zed2_sensors
  PUBLIC
  Obelisk::Core
  Obelisk::Zoo
  Zed2SensorsInterface)
ament_target_dependencies(zed2_sensors
  PUBLIC
  rclcpp
  rclcpp_lifecycle)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  zed2_sensors
  DESTINATION lib/${PROJECT_NAME})

ament_package()
