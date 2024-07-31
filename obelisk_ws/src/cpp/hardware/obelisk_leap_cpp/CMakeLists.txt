cmake_minimum_required(VERSION 3.16)
project(obelisk_leap_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)

# Get the Obelisk library
set(OBELISK_BUILD_LEAP ON)
include(FetchContent)
FetchContent_Declare(
  Obelisk
  SOURCE_DIR $ENV{OBELISK_ROOT}/obelisk/cpp
)

# Lets us get Obelisk::Core and Obelisk::Zoo
FetchContent_MakeAvailable(Obelisk)

# ObeliskLeapRobot
add_executable(ObeliskLeapRobot src/obelisk_leap_robot.cpp)
target_link_libraries(ObeliskLeapRobot PUBLIC Obelisk::Core Obelisk::Zoo)
ament_target_dependencies(ObeliskLeapRobot PUBLIC rclcpp)
ament_target_dependencies(ObeliskLeapRobot PUBLIC dynamixel_sdk)

# LeapExampleController
add_executable(LeapExampleController src/leap_example_controller.cpp)
target_link_libraries(LeapExampleController PUBLIC Obelisk::Core Obelisk::Zoo)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  ObeliskLeapRobot
  DESTINATION lib/${PROJECT_NAME})

  install(TARGETS
  LeapExampleController
  DESTINATION lib/${PROJECT_NAME})

ament_package()
