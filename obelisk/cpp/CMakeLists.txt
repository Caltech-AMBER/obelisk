cmake_minimum_required(VERSION 3.16)

project(ObeliskCpp
    VERSION 0.0.1
    LANGUAGES CXX
    DESCRIPTION "Obelisk c++ utils")

message(STATUS "Building obelisk_cpp")

set(CMAKE_CXX_STANDARD 17 CACHE STRING "The C++ standard to use")

add_subdirectory(obelisk_cpp)
add_subdirectory(zoo)

if (CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    include(CTest)
endif()

if (CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND BUILD_TESTING)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../tests/tests_cpp ${CMAKE_CURRENT_BINARY_DIR}/tests)
endif()
