cmake_minimum_required(VERSION 3.8)
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
  ObeliskCpp
  SOURCE_DIR $ENV{OBELISK_ROOT}/obelisk/cpp/obelisk_cpp
)

FetchContent_MakeAvailable(ObeliskCpp)

add_executable(obelisk_mujoco_robot src/obelisk_mujoco_robot.cpp)
target_link_libraries(obelisk_mujoco_robot PUBLIC ObeliskCpp)

ament_target_dependencies(obelisk_mujoco_robot PUBLIC rclcpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  obelisk_mujoco_robot
  DESTINATION lib/${PROJECT_NAME})

ament_package()