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

# Create a wrapper target for ZED libraries to disable warnings
add_library(ZedNoWarnings INTERFACE)
target_link_libraries(ZedNoWarnings INTERFACE ${ZED_LIBS})
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(ZedNoWarnings INTERFACE -w)
elseif(MSVC)
    target_compile_options(ZedNoWarnings INTERFACE /W0)
endif()

# ----------- YAML ----------- #
# Fetch yaml-cpp; disable tests and tools, which are unnecessary and can throw warnings
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
set(YAML_CPP_BUILD_TESTS OFF)
set(YAML_CPP_BUILD_TOOLS OFF)
FetchContent_Declare(
    yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG master  # [June 10, 2025] The 0.8.0 release needs to have the minimum version raised to 3.5, master fixed
)
FetchContent_MakeAvailable(yaml-cpp)

# Disable warnings for yaml-cpp
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(yaml-cpp PRIVATE -w -Wno-unused-parameter)
elseif(MSVC)
    target_compile_options(yaml-cpp PRIVATE /W0 /wd4100)
endif()

# ----------- Making the Libraries ----------- #
# ZED2 sensor
add_library(Zed2SensorsInterface INTERFACE)
target_include_directories(Zed2SensorsInterface INTERFACE
    ${CUDA_INCLUDE_DIRS}
    ${ZED_INCLUDE_DIRS}
)

target_link_libraries(Zed2SensorsInterface INTERFACE
    Eigen3::Eigen
    ${ZED_LIBS}
    Obelisk::Utils
    yaml-cpp
    ZedNoWarnings)

ament_target_dependencies(Zed2SensorsInterface
    INTERFACE
    rclcpp
    rclcpp_lifecycle
)
