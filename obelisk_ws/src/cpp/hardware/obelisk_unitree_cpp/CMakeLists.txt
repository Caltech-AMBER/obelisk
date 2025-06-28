cmake_minimum_required(VERSION 3.16)
project(obelisk_unitree_cpp)

if(DEFINED ENV{OBELISK_BUILD_UNITREE} AND "$ENV{OBELISK_BUILD_UNITREE}" STREQUAL "true")
    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
    endif()

    # find dependencies
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)

    # Get the Obelisk library
    include(FetchContent)
    FetchContent_Declare(
    Obelisk
    SOURCE_DIR $ENV{OBELISK_ROOT}/obelisk/cpp
    )

    # Lets us get Obelisk::Core and Obelisk::Zoo
    FetchContent_MakeAvailable(Obelisk)

    # obelisk_unitree_sim
    add_executable(obelisk_unitree_sim src/unitree_sim_robot.cpp)
    target_link_libraries(obelisk_unitree_sim PUBLIC Obelisk::Core Obelisk::Zoo)

    # obelisk_unitree_d1_hardware
    add_executable(unitree_d1_hardware src/unitree_d1_hardware.cpp)
    target_link_libraries(unitree_d1_hardware PUBLIC Obelisk::Core UnitreeInterface)

    # obelisk_unitree_g1_hardware
    add_executable(obelisk_unitree_g1_hardware src/unitree_g1_hardware_robot.cpp)
    target_link_libraries(obelisk_unitree_g1_hardware PUBLIC Obelisk::Core UnitreeInterface)

    # obelisk_unitree_go2_hardware
    add_executable(obelisk_unitree_go2_hardware src/unitree_go2_hardware_robot.cpp)
    target_link_libraries(obelisk_unitree_go2_hardware PUBLIC Obelisk::Core UnitreeInterface)

    # unitree_example_controller
    add_executable(unitree_example_controller src/g1_example_controller.cpp)
    target_link_libraries(unitree_example_controller PUBLIC Obelisk::Core Obelisk::Zoo)

    # unitree_go2_estimator
    add_executable(unitree_go2_estimator src/unitree_go2_estimator.cpp)
    target_link_libraries(unitree_go2_estimator PUBLIC Obelisk::Core Obelisk::Zoo)

    # unitree_example_estimator
    add_executable(unitree_example_estimator src/g1_example_estimator.cpp)
    target_link_libraries(unitree_example_estimator PUBLIC Obelisk::Core Obelisk::Zoo)

    # unitree_joystick
    add_executable(obelisk_unitree_joystick src/unitree_joystick.cpp)
    target_link_libraries(obelisk_unitree_joystick PUBLIC Obelisk::Core Obelisk::Zoo)

    if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
    endif()

    install(TARGETS
        obelisk_unitree_sim
        DESTINATION lib/${PROJECT_NAME})

    install(TARGETS
        obelisk_unitree_g1_hardware
        DESTINATION lib/${PROJECT_NAME})

    install(TARGETS
        obelisk_unitree_go2_hardware
        DESTINATION lib/${PROJECT_NAME})

    install(TARGETS
        unitree_example_controller
        DESTINATION lib/${PROJECT_NAME})

    install(TARGETS
        unitree_example_estimator
        DESTINATION lib/${PROJECT_NAME})

    install(TARGETS
        unitree_go2_estimator
        DESTINATION lib/${PROJECT_NAME})

    install(TARGETS
        obelisk_unitree_joystick
        DESTINATION lib/${PROJECT_NAME})

    ament_package()
endif()

