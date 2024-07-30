cmake_minimum_required(VERSION 3.16)
project(obelisk_sensing_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Get the Obelisk library
include(FetchContent)
FetchContent_Declare(
  Obelisk
  SOURCE_DIR $ENV{OBELISK_ROOT}/obelisk/cpp
)

# Lets us Get Obelisk::Core and Obelisk::Zoo
FetchContent_MakeAvailable(Obelisk)

# zed2_sensors

# TODO(ahl): do I need all of this?
###############################################################################
option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)
find_package(ZED 4 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})
###############################################################################

add_executable(zed2_sensor src/zed2_sensor.cpp)

# TODO(ahl): why do I need ZED_LIBRARIES here?
target_link_libraries(zed2_sensor
  PUBLIC
  Obelisk::Core
  Obelisk::Zoo
  ${ZED_LIBRARIES})
ament_target_dependencies(zed2_sensor
  PUBLIC
  rclcpp
  rclcpp_lifecycle)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
