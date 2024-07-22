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
include(FetchContent)
FetchContent_Declare(
  Obelisk
  SOURCE_DIR $ENV{OBELISK_ROOT}/obelisk/cpp
)

# Lets us get Obelisk::Core and Obelisk::Zoo
FetchContent_MakeAvailable(Obelisk)

set(LD_LIBRARY_PATH "/home/gavin/caltech/surf2024/obelisk/.pixi/envs/dev/lib:$ENV{LD_LIBRARY_PATH}")
include_directories("${dynamixel_sdk_INCLUDE_DIRS}/dynamixel_sdk")

# export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/gavin/caltech/surf2024/obelisk/.pixi/envs/dev/lib/

add_executable(obelisk_leap_robot src/obelisk_leap_robot.cpp)
target_link_libraries(obelisk_leap_robot PUBLIC Obelisk::Core)
target_link_libraries(obelisk_leap_robot PUBLIC Obelisk::Zoo)

ament_target_dependencies(obelisk_leap_robot PUBLIC rclcpp)
ament_target_dependencies(obelisk_leap_robot PUBLIC dynamixel_sdk)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  obelisk_leap_robot
  DESTINATION lib/${PROJECT_NAME})

ament_package()
