message(STATUS "Configuring Obelisk Cpp")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------- ROS 2 Packages ------- #
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcl REQUIRED)
find_package(urdf REQUIRED)
find_package(sensor_msgs REQUIRED)

# ------- Obelisk Messages ------- #
find_package(obelisk_control_msgs REQUIRED)
find_package(obelisk_sensor_msgs REQUIRED)
find_package(obelisk_estimator_msgs REQUIRED)


# ------- Source files ------- #
set(VIZ_INC "${CMAKE_CURRENT_SOURCE_DIR}/include")


# ------- Making the library ------- #
add_library(ObkViz INTERFACE)
add_library(Obelisk::Viz ALIAS ObkViz)   # Namespaced alias
target_include_directories(ObkViz INTERFACE ${VIZ_INC})

target_link_libraries(ObkViz INTERFACE Obelisk::Core)

ament_target_dependencies(ObkViz INTERFACE
  rclcpp
  rclcpp_lifecycle
  obelisk_control_msgs
  obelisk_estimator_msgs
  obelisk_sensor_msgs
  std_msgs
  urdf
  sensor_msgs)
