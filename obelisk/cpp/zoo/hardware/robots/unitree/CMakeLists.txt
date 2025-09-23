cmake_minimum_required(VERSION 3.16)
project(unitree_interface LANGUAGES CXX)

message(STATUS "Configuring Unitree Interface")

find_package(ament_cmake REQUIRED)

include(FetchContent)

# Let `unitree_sdk` refer to the content fetched from the git repo.
FetchContent_Declare(
    unitree_sdk
    GIT_REPOSITORY https://github.com/unitreerobotics/unitree_sdk2.git
    GIT_TAG 7b00ca4b766272d137cf7ff751d4a067bc99ca85) # Most recent commit includes the Write()
    # member function in channel_publisher.hpp

    # GIT_TAG 3a4680ae9b00df59e60f7e63cfb0fcc432a9d08d # This commit doesn't include the aforementioned
    # member function, however, it has the SwitchGait() member function which enables go2_interface.h to compile.

# Couldn't find `UNITREE_SDK_BUILD_EXAMPLES` being defined in unitree_sdk2.
# set(UNITREE_SDK_BUILD_EXAMPLES OFF CACHE BOOL "Disable building examples")
# `BUILD_EXAMPLES` is declared in unitree_sdk2 repo's CMakeLists.txt 
set(BUILD_EXAMPLES OFF CACHE BOOL "Disable building examples")

FetchContent_MakeAvailable(unitree_sdk)

# Adds an Object Library to compile source files without linking their 
# object files into a library
add_library(UnitreeInterface INTERFACE)

# Adds the current directory (.) to the include path for the UnitreeInterface library
target_include_directories(UnitreeInterface INTERFACE .)

# Specifies that UnitreeInterface depends on two libraries: Obelisk::Core and unitree_sdk2.
# unitree_sdk2 is the CMake target name defined by unitree_sdk2 repo's CMakeLists.txt.
target_link_libraries(UnitreeInterface INTERFACE 
  Obelisk::Core 
  unitree_sdk2 
)

# Adds ROS 2 dependencies to UnitreeInterface
ament_target_dependencies(UnitreeInterface INTERFACE
  rclcpp
  rclcpp_lifecycle
  sensor_msgs)

# add_executable(unitree_test unitree_test.cpp)
# target_link_libraries(unitree_test unitree_sdk2)

# TODO:
# - Run without Pixi
# - Install Eigen and Ping