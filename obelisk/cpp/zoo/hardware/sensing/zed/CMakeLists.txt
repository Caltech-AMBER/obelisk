cmake_minimum_required(VERSION 3.16)
project(zed2_sensors LANGUAGES CXX)

message(STATUS "Configuring Zed2 Sensors")

# ----------- FetchContent ----------- #
include(FetchContent)

# ----------- ROS ----------- #
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Eigen3 REQUIRED)

# ----------- ZED ----------- #
# [NOTE] for this to build properly, it is necessary for CMAKE_PREFIX_PATH to include the location of cuda
# in pixi, this is under $PIXI_PROJECT_ROOT/.pixi/envs/$PIXI_ENVIRONMENT_NAME/targets/x86_64-linux
# for local install, it should be under /usr/local/cuda or /usr/local/cuda-{MAJOR_VERSION}.{MINOR_VERSION}
find_package(ZED 4 REQUIRED)
link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})
set(ZED_LIBS
    ${ZED_LIBRARIES}
    ${CUDA_LIBRARIES}
)

# ----------- YAML ----------- #
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
FetchContent_Declare(
        yaml-cpp
        GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
        GIT_TAG 0.8.0 # note all other versions are of the form yaml-cpp-x.y.z
)
FetchContent_MakeAvailable(yaml-cpp)


# ----------- Making the executables ----------- #
# ZED2 sensor
add_library(Zed2SensorsInterface INTERFACE)
target_include_directories(Zed2SensorsInterface INTERFACE
    ${CUDA_INCLUDE_DIRS}
    ${ZED_INCLUDE_DIRS}
)
target_link_libraries(Zed2SensorsInterface INTERFACE
    yaml-cpp
    Eigen3::Eigen
    ${ZED_LIBS}
    Obelisk::Utils)

ament_target_dependencies(Zed2SensorsInterface
    INTERFACE
    rclcpp
    rclcpp_lifecycle
)
