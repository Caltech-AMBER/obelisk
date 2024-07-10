cmake_minimum_required(VERSION 3.16)

project(obelisk_cpp
    VERSION 0.0.1
    LANGUAGES CXX
    DESCRIPTION "Obelisk c++ utils")

message(STATUS "Building obelisk_cpp")

set(CMAKE_CXX_STANDARD 17 CACHE STRING "The C++ standard to use")

add_subdirectory(obelisk_cpp)
add_subdirectory(zoo)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../tests/tests_cpp ${CMAKE_CURRENT_BINARY_DIR}/tests)
