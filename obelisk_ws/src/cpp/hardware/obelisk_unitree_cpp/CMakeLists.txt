cmake_minimum_required(VERSION 3.16)
project(obelisk_unitree_cpp)

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

# Lets us get Obelisk::Core and Obelisk::Zoo
FetchContent_MakeAvailable(Obelisk)

# TODO: Add in the hardware node
# obelisk_g1_robot
add_executable(obelisk_g1_robot src/unitree_sim_robot.cpp)
target_link_libraries(obelisk_g1_robot PUBLIC Obelisk::Core Obelisk::Zoo)

# unitree_example_controller
add_executable(unitree_example_controller src/g1_example_controller.cpp)
target_link_libraries(unitree_example_controller PUBLIC Obelisk::Core Obelisk::Zoo)

# unitree_example_estimator
add_executable(unitree_example_estimator src/g1_example_estimator.cpp)
target_link_libraries(unitree_example_estimator PUBLIC Obelisk::Core Obelisk::Zoo)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
    obelisk_g1_robot
    DESTINATION lib/${PROJECT_NAME})

install(TARGETS
    unitree_example_controller
    DESTINATION lib/${PROJECT_NAME})

install(TARGETS
    unitree_example_estimator
    DESTINATION lib/${PROJECT_NAME})

ament_package()
