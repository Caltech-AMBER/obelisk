cmake_minimum_required(VERSION 3.8)
project(obelisk_sim_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# ------- Mujoco ------- #
# Setup Mujoco path
set(MUJOCO_VERSION "mujoco-3.1.6" CACHE STRING "mujoco version")
set(MUJOCO_DIR "/home/$ENV{USER}/mujoco/${MUJOCO_VERSION}" CACHE PATH "Path to Mujoco dir")
message(STATUS "mujoco directory: ${MUJOCO_DIR}")

# Setup the Mujoco library
find_library(MUJOCO_LIB mujoco REQUIRED HINTS ${MUJOCO_DIR}/lib)
message(STATUS ${MUJOCO_LIB})
set_property(TARGET ${MUJOCO_LIB} PROPERTY IMPORTED_LOCATION ${MUJOCO_DIR}/lib/libmujoco.so)

# target_link_libraries(<MY_TARGET> PUBLIC mujoco)

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

ament_package()
