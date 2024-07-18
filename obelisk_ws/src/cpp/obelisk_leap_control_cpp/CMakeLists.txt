cmake_minimum_required(VERSION 3.8)
project(obelisk_leap_control_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)


# Get the Obelisk library
include(FetchContent)
FetchContent_Declare(
  Obelisk
  SOURCE_DIR $ENV{OBELISK_ROOT}/obelisk/cpp
)

# Lets us Get Obelisk::Core and Obelisk::Zoo
FetchContent_MakeAvailable(Obelisk)

add_executable(obelisk_leap_controller src/obelisk_leap_controller.cpp)
target_link_libraries(obelisk_leap_controller PUBLIC Obelisk::Core)
target_link_libraries(obelisk_leap_controller PUBLIC Obelisk::Zoo)

target_link_libraries(obelisk_leap_controller
  PUBLIC
  rosidl_runtime_c::rosidl_runtime_c
  rosidl_runtime_cpp::rosidl_runtime_cpp
)

ament_target_dependencies(obelisk_leap_controller PUBLIC
  "rosidl_typesupport_cpp"
)


ament_target_dependencies(obelisk_leap_controller PUBLIC rclcpp)


target_link_libraries(obelisk_leap_controller PUBLIC
  ${rosidl_typesupport_cpp_LIBRARIES}
)

# =========================
# !!!RUN THIS LINE BEFORE RUNNING THE LEAP HAND!!!
# This is necessary to load rosidl_typesupport_cpp for some reason

# export LD_LIBRARY_PATH=/home/gavin/caltech/surf2024/obelisk/.pixi/envs/dev/lib:$LD_LIBRARY_PATH
# =========================

set_property(TARGET obelisk_leap_controller APPEND PROPERTY
    ENVIRONMENT "LD_LIBRARY_PATH=$ENV{OBELISK_ROOT}/.pixi/envs/dev/lib:$ENV{LD_LIBRARY_PATH}")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  obelisk_leap_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()
