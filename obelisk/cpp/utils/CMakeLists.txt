message(STATUS "Configuring Obelisk Utils")


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------- ROS 2 Packages ------- #
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcl REQUIRED)

# ------- Obelisk Messages ------- #
find_package(obelisk_std_msgs REQUIRED)

# ------- Eigen ------- #
find_package(Eigen3 REQUIRED)

# ------- Source files ------- #
set(UTILS_INC "${CMAKE_CURRENT_SOURCE_DIR}/include")

# ------- Making the library ------- #
add_library(ObkUtils INTERFACE)
add_library(Obelisk::Utils ALIAS ObkUtils)   # Namespaced alias
target_include_directories(ObkUtils INTERFACE ${UTILS_INC} ${mujoco_SOURCE_DIR}/include)

target_link_libraries(ObkUtils INTERFACE Eigen3::Eigen)

ament_target_dependencies(ObkUtils INTERFACE
  rclcpp
  rclcpp_lifecycle
  obelisk_std_msgs
  std_msgs)
