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

# Zed2Sensors
add_executable(Zed2Sensors src/zed2_sensors.cpp)
target_link_libraries(Zed2Sensors
  PUBLIC
  Obelisk::Core
  Obelisk::Zoo
  Zed2SensorsInterface)
ament_target_dependencies(Zed2Sensors
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
  Zed2Sensors
  DESTINATION lib/${PROJECT_NAME})

ament_package()
