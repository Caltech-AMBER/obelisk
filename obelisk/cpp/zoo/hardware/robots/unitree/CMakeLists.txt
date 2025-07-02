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
    # method in channel_publisher.hpp

    # GIT_TAG 3a4680ae9b00df59e60f7e63cfb0fcc432a9d08d # This commit doesn't include the aforementioned
    # method, however, it has the SwitchGait() method which enables go2_interface.h to compile.

# Couldn't find `UNITREE_SDK_BUILD_EXAMPLES` being defined in unitree_sdk2.
# set(UNITREE_SDK_BUILD_EXAMPLES OFF CACHE BOOL "Disable building examples")
# `BUILD_EXAMPLES` is declared in unitree_sdk2 repo's CMakeLists.txt 
set(BUILD_EXAMPLES OFF CACHE BOOL "Disable building examples")

FetchContent_MakeAvailable(unitree_sdk)

# Get the .cpp files in msg and store it into the `UNITREE_ARM_MSGS`. A library should include
# solely .cpp files.
set(UNITREE_ARM_MSGS
  msg/ArmString_.cpp
)

# Creates a library called `unitree_arm_msgs` containing all files in `UNITREE_ARM_MSGS`
add_library(unitree_arm_msgs SHARED ${UNITREE_ARM_MSGS})

target_include_directories(unitree_arm_msgs PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/msg # The compiler needs ${CMAKE_CURRENT_SOURCE_DIR}/msg 
  # in the include path to find ArmString_.hpp when compiling ArmString_.cpp
  /usr/local/include/ddscxx 
  /usr/local/include/iceoryx/v2.0.2 # FIXME: not sure if this is necessary. 
  # It's in d1_sdk's CMakeLists.txt, but I can't find the folder on the computer.
)

# Link `unitree_arm_msgs` to other libraries that d1_sdk depends on. These other libraries
# are listed in d1_sdk's CMakeLists.txt.
target_link_libraries(unitree_arm_msgs PUBLIC
  unitree_sdk2 ddsc ddscxx rt pthread
)

# Adds an Object Library to compile source files without linking their 
# object files into a library
add_library(UnitreeInterface INTERFACE)

# Adds the current directory (.) to the include path for the UnitreeInterface library
target_include_directories(UnitreeInterface INTERFACE .
${CMAKE_CURRENT_SOURCE_DIR}/msg
)

# Specifies that UnitreeInterface depends on two libraries: Obelisk::Core and unitree_sdk2.
# unitree_sdk2 is the CMake target name defined by unitree_sdk2 repo's CMakeLists.txt.
target_link_libraries(UnitreeInterface INTERFACE 
  Obelisk::Core 
  unitree_sdk2 
  unitree_arm_msgs
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