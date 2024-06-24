#include <catch2/catch_test_macros.hpp>
#include <iostream>

#include "obelisk_node.h"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

namespace obelisk {
    // Helper class for testing
    class ObeliskNodeTester : public ObeliskNode {
       public:
        ObeliskNodeTester()
            : ObeliskNode("obelisk_tester") {}

        // Templated generic callback to facilitate easy testing
        template <typename MessageT>
        void GenericCallback(const MessageT& msg) {}
    };
}  // namespace obelisk

TEST_CASE("Obelisk Node Pub and Sub", "[obelisk_node]") {
    rclcpp::init(0, nullptr);

    obelisk::ObeliskNodeTester node;

    SECTION("Publishers") {
        // Verify that a dissalowed message throws an error
        REQUIRE_THROWS(
            node.create_publisher<std_msgs::msg::String>("topic", 10));

        // Verify that allowed messages are fine
        REQUIRE_NOTHROW(
            node.create_publisher<obelisk_control_msgs::msg::PositionSetpoint>(
                "topic", 10));
        REQUIRE_NOTHROW(
            node.create_publisher<obelisk_estimator_msgs::msg::EstimatedState>(
                "topic", 10));
        REQUIRE_NOTHROW(
            node.create_publisher<obelisk_sensor_msgs::msg::JointEncoder>(
                "topic", 10));
        REQUIRE_NOTHROW(
            node.create_publisher<obelisk_sensor_msgs::msg::JointEncoders>(
                "topic", 10));
        REQUIRE_NOTHROW(
            node.create_publisher<obelisk_sensor_msgs::msg::TrueSimState>(
                "topic", 10));
    }

    SECTION("Subscribers") {
        // Verify that allowed messages are fine
        REQUIRE_NOTHROW(
            node.create_subscription<
                obelisk_control_msgs::msg::PositionSetpoint>("topic", 10,
                std::bind(&obelisk::ObeliskNodeTester::GenericCallback<
                              obelisk_control_msgs::msg::PositionSetpoint>,
                    &node, _1)));
        REQUIRE_NOTHROW(
            node.create_subscription<
                obelisk_estimator_msgs::msg::EstimatedState>("topic", 10,
                std::bind(&obelisk::ObeliskNodeTester::GenericCallback<
                              obelisk_estimator_msgs::msg::EstimatedState>,
                    &node, _1)));
        REQUIRE_NOTHROW(
            node.create_subscription<obelisk_sensor_msgs::msg::JointEncoders>(
                "topic", 10,
                std::bind(&obelisk::ObeliskNodeTester::GenericCallback<
                              obelisk_sensor_msgs::msg::JointEncoders>,
                    &node, _1)));
        REQUIRE_NOTHROW(
            node.create_subscription<obelisk_sensor_msgs::msg::TrueSimState>(
                "topic", 10,
                std::bind(&obelisk::ObeliskNodeTester::GenericCallback<
                              obelisk_sensor_msgs::msg::TrueSimState>,
                    &node, _1)));
        REQUIRE_NOTHROW(
            node.create_subscription<obelisk_sensor_msgs::msg::JointEncoder>(
                "topic", 10,
                std::bind(&obelisk::ObeliskNodeTester::GenericCallback<
                              obelisk_sensor_msgs::msg::JointEncoder>,
                    &node, _1)));
        REQUIRE_NOTHROW(
            node.create_subscription<rcl_interfaces::msg::ParameterEvent>(
                "topic", 10,
                std::bind(&obelisk::ObeliskNodeTester::GenericCallback<
                              rcl_interfaces::msg::ParameterEvent>,
                    &node, _1)));

        // Verify that a dissalowed message throws an error
        REQUIRE_THROWS(
            node.create_subscription<std_msgs::msg::String>("topic", 10,
                std::bind(&obelisk::ObeliskNodeTester::GenericCallback<
                              std_msgs::msg::String>,
                    &node, _1)));
    }

    rclcpp::shutdown();
}

TEST_CASE("Subscriber from string", "[obelisk_node]") {
    rclcpp::init(0, nullptr);

    obelisk::ObeliskNodeTester node;

    node.CreateSubscriptionFromConfigStr("test1");

    rclcpp::shutdown();
}
