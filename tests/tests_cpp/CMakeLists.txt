message(STATUS "Configuring obelisk_cpp tests")

# ------- Catch2 for testing ------- #
include(FetchContent)

FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.5.4 # or a later release
)

FetchContent_MakeAvailable(Catch2)

# Copy additional test data to the build directory
file(COPY test_data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

add_executable(NodeTest
    test_node.cpp
    test_controller.cpp
    test_estimator.cpp
    test_sensor.cpp
    test_robot.cpp
    test_sim_robot.cpp
    test_mujoco_sim_robot.cpp
    claude_obelisk_node_test.cpp
    claude_obelisk_controller_test.cpp
    claude_obelisk_sim_robot_test.cpp
    claude_obelisk_sensor_test.cpp
    claude_obelisk_robot_test.cpp
    claude_obelisk_estimator_test.cpp)

add_executable(VizTest
    test_viz_robot.cpp
    test_viz_robot_default.cpp
    )

target_link_libraries(NodeTest PRIVATE Catch2::Catch2WithMain)
target_link_libraries(NodeTest PUBLIC Obelisk::Core)


target_link_libraries(VizTest PRIVATE Catch2::Catch2WithMain)
target_link_libraries(VizTest PUBLIC Obelisk::Core Obelisk::Viz)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
ament_target_dependencies(NodeTest PUBLIC rclcpp std_msgs)

include(Catch)
catch_discover_tests(NodeTest)
catch_discover_tests(VizTest)
