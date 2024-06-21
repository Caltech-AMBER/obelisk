message(STATUS "Building obelisk_cpp tests")

# Catch2 for testing
include(FetchContent)

FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.5.4 # or a later release
)

FetchContent_MakeAvailable(Catch2)

# Copy additional test data to the build directory
file(COPY test_data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

add_executable(dummy_test
    dummy_test.cpp
    dummy_test_2.cpp)
target_link_libraries(dummy_test PRIVATE Catch2::Catch2WithMain)
target_link_libraries(dummy_test PUBLIC lib1)

include(Catch)
include(CTest)
catch_discover_tests(dummy_test)
