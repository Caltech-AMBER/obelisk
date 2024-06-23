#include <catch2/catch_test_macros.hpp>
#include <iostream>

#include "obelisk_node.h"

TEST_CASE("Obelisk Node Tests", "[obelisk_node]") {
    std::cout << "Hello3." << std::endl;
    rclcpp::init(0, nullptr);

    obelisk::ObeliskNode("hello_world2");

    rclcpp::shutdown();

    REQUIRE(1 == 1);
}
