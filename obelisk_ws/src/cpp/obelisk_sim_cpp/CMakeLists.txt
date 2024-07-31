cmake_minimum_required(VERSION 3.16)
project(obelisk_sim_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Get the Obelisk library
include(FetchContent)
FetchContent_Declare(
  Obelisk
  SOURCE_DIR $ENV{OBELISK_ROOT}/obelisk/cpp
)

# Lets us Get Obelisk::Core and Obelisk::Zoo
FetchContent_MakeAvailable(Obelisk)

# ObeliskMujocoRobot
add_executable(ObeliskMujocoRobot src/obelisk_mujoco_robot.cpp)
target_link_libraries(ObeliskMujocoRobot PUBLIC Obelisk::Core)

ament_target_dependencies(ObeliskMujocoRobot PUBLIC rclcpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
ObeliskMujocoRobot
  DESTINATION lib/${PROJECT_NAME})

ament_package()
