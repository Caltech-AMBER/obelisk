#include <catch2/catch_test_macros.hpp>
#include <iostream>

#include "obelisk_node.h"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"

TEST_CASE("Obelisk Node Tests", "[obelisk_node]") {
    std::cout << "Hello3." << std::endl;
    rclcpp::init(0, nullptr);

    rclcpp_lifecycle::LifecycleNode node("hello_world");
    node.create_publisher<std_msgs::msg::String>("topic", 10);

    obelisk::ObeliskNode node2("hello_world2");
    node2.create_publisher<std_msgs::msg::String>("topic", 10);

    rclcpp::shutdown();

    REQUIRE(1 == 1);
}
