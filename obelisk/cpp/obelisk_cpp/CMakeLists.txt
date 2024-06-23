message(STATUS "Building Obelisk Cpp")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcl REQUIRED)

set(OBELISK_CPP_INC "${obelisk_cpp_SOURCE_DIR}/obelisk_cpp/include")
set(OBELISK_CPP_HEADER_LIST
        "${OBELISK_CPP_INC}/obelisk_node.h")

add_library(ObeliskCpp ${OBELISK_CPP_HEADER_LIST})
set_target_properties(ObeliskCpp PROPERTIES LINKER_LANGUAGE CXX)

target_include_directories(ObeliskCpp PUBLIC
    $<BUILD_INTERFACE:${OBELISK_CPP_INC}>
    $<INSTALL_INTERFACE:$obelisk_cpp/include>)

target_link_libraries(ObeliskCpp PUBLIC rclcpp::rclcpp)
target_link_libraries(ObeliskCpp PUBLIC rclcpp_lifecycle::rclcpp_lifecycle)
