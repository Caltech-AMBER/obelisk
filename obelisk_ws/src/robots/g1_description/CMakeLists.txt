cmake_minimum_required(VERSION 3.16)
project(g1_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# install all the mesh files
install(
  DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

# install the urdf
install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# install the mujoco files
install(
  DIRECTORY mujoco/
  DESTINATION share/${PROJECT_NAME}/mujoco
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
