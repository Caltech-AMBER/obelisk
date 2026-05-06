cmake_minimum_required(VERSION 3.16)
# Read from environment variable, default to ON if not set
if(DEFINED ENV{OBELISK_USE_MUJOCO})
  set(USE_MUJOCO_DEFAULT $ENV{OBELISK_USE_MUJOCO})
else()
  set(USE_MUJOCO_DEFAULT ON)
endif()

option(USE_MUJOCO "Enable Mujoco support" ${USE_MUJOCO_DEFAULT})

if(USE_MUJOCO)
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

  # obelisk_mujoco_robot
  add_executable(obelisk_mujoco_robot src/obelisk_mujoco_robot.cpp)
  target_link_libraries(obelisk_mujoco_robot PUBLIC Obelisk::Core)

  ament_target_dependencies(obelisk_mujoco_robot PUBLIC rclcpp)

  # Demo the raycaster
  # Fetch yaml-cpp; disable tests and tools, which are unnecessary and can throw warnings
  set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
  set(YAML_CPP_BUILD_TESTS OFF)
  set(YAML_CPP_BUILD_TOOLS OFF)
  FetchContent_Declare(
      yaml-cpp
      GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
      GIT_TAG 0.8.0 # note all other versions are of the form yaml-cpp-x.y.z
  )
  FetchContent_MakeAvailable(yaml-cpp)

  # Disable warnings for yaml-cpp
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
      target_compile_options(yaml-cpp PRIVATE -w -Wno-unused-parameter)
  elseif(MSVC)
      target_compile_options(yaml-cpp PRIVATE /W0 /wd4100)
  endif()
  add_executable(ray_caster_demo src/ray_caster_demo.cpp)
  find_package(Eigen3 REQUIRED)
  target_link_libraries(ray_caster_demo PUBLIC Obelisk::Core Eigen3::Eigen yaml-cpp)

  if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
  endif()

  install(TARGETS
    obelisk_mujoco_robot
    DESTINATION lib/${PROJECT_NAME})

  ament_package()
endif()
