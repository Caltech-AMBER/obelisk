cmake_minimum_required(VERSION 3.16)
project(obelisk_westwood_cpp)

if(DEFINED ENV{OBELISK_BUILD_WESTWOOD} AND "$ENV{OBELISK_BUILD_WESTWOOD}" STREQUAL "true")
    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_compile_options(-Wall -Wextra -Wpedantic)
    endif()

    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)

    # Pull in Obelisk core + hardware libraries (incl. WestwoodInterface)
    include(FetchContent)
    FetchContent_Declare(
        Obelisk
        SOURCE_DIR $ENV{OBELISK_ROOT}/obelisk/cpp
    )
    FetchContent_MakeAvailable(Obelisk)

    # Optional Mujoco sim — matches obelisk_unitree_cpp/CMakeLists.txt pattern
    if(DEFINED ENV{OBELISK_USE_MUJOCO})
        set(USE_MUJOCO_DEFAULT $ENV{OBELISK_USE_MUJOCO})
    else()
        set(USE_MUJOCO_DEFAULT ON)
    endif()
    option(USE_MUJOCO "Enable Mujoco support" ${USE_MUJOCO_DEFAULT})

    # ── hardware robot ──
    add_executable(obelisk_themis_hardware src/themis_hardware_robot.cpp)
    target_link_libraries(obelisk_themis_hardware PUBLIC Obelisk::Core WestwoodInterface)

    # ── example controller (joint sweep stub) ──
    add_executable(themis_example_controller src/themis_example_controller.cpp)
    target_link_libraries(themis_example_controller PUBLIC Obelisk::Core Obelisk::Hardware)

    # ── Mujoco sim driver ──
    if(USE_MUJOCO)
        add_executable(obelisk_themis_sim src/themis_sim_robot.cpp)
        target_link_libraries(obelisk_themis_sim PUBLIC Obelisk::Core Obelisk::Hardware)
    endif()

    install(TARGETS obelisk_themis_hardware
        DESTINATION lib/${PROJECT_NAME})
    install(TARGETS themis_example_controller
        DESTINATION lib/${PROJECT_NAME})
    if(USE_MUJOCO)
        install(TARGETS obelisk_themis_sim
            DESTINATION lib/${PROJECT_NAME})
    endif()

    # Install the on-robot Python bridge so users can find it via the install tree.
    # The server itself is GPL-3.0 (see server/LICENSE); the C++ targets above are MIT.
    install(DIRECTORY server/
        DESTINATION share/${PROJECT_NAME}/server)

    if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        set(ament_cmake_copyright_FOUND TRUE)
        set(ament_cmake_cpplint_FOUND TRUE)
        ament_lint_auto_find_test_dependencies()
    endif()

    ament_package()
endif()
