#include <catch2/catch_test_macros.hpp>
#include <iostream>

#include "obelisk_controller.h"
#include "obelisk_node.h"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

namespace obelisk {
// Helper class for testing
class ObeliskNodeTester : public ObeliskNode {
  public:
    ObeliskNodeTester() : ObeliskNode("obelisk_tester") {}

    template <typename MessageT>
    void SubscriptionConfigStrTester(const std::string& config) {
        CreateSubscriptionFromConfigStr<MessageT>(
            config,
            std::bind(&ObeliskNodeTester::GenericCallback<MessageT>, this, _1));
    }

    template <typename MessageT>
    void PublishConfigStrTester(const std::string& config) {
        CreatePublisherFromConfigStr<MessageT>(config);
    }

    void TimerConfigFromStrTester(const std::string& config) {
        CreateWallTimerFromConfigStr(
            config, std::bind(&ObeliskNodeTester::SimpleCallback, this));
    }
    void SimpleCallback() {}

    // Templated generic callback to facilitate easy testing
    template <typename MessageT> void GenericCallback(const MessageT& msg) {}
};

class ObeliskControllerTester
    : public ObeliskController<
          obelisk::MessagePack<obelisk_control_msgs::msg::PositionSetpoint>,
          obelisk::MessagePack<obelisk_estimator_msgs::msg::EstimatedState>> {
  public:
    ObeliskControllerTester() : ObeliskController("obelisk_controller_tester") {
        // std::get<ObeliskController::PublisherType<obelisk_control_msgs::msg::PositionSetpoint>>(control_publishers_)
        //     =
        //     CreatePublisherFromConfigStr<obelisk_control_msgs::msg::PositionSetpoint>("topic:topic1");
        CreateAllControllerPublishers();
    }
};
} // namespace obelisk

// -------------------------------------------------- //
// --------------- Obelisk Node Tests --------------- //
// -------------------------------------------------- //
TEST_CASE("Obelisk Node Pub and Sub", "[obelisk_node]") {
    rclcpp::init(0, nullptr);

    obelisk::ObeliskNodeTester node;

    SECTION("Publishers") {
        // Verify that a dissalowed message throws an error
        REQUIRE_THROWS(
            node.create_publisher<std_msgs::msg::String>("topic", 10));

        // Try with the override flag
        REQUIRE_NOTHROW(
            node.create_publisher<std_msgs::msg::String>("topic", 10, true));

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
        REQUIRE_NOTHROW(node.create_subscription<
                        obelisk_control_msgs::msg::PositionSetpoint>(
            "topic", 10,
            std::bind(&obelisk::ObeliskNodeTester::GenericCallback<
                          obelisk_control_msgs::msg::PositionSetpoint>,
                      &node, _1)));
        REQUIRE_NOTHROW(node.create_subscription<
                        obelisk_estimator_msgs::msg::EstimatedState>(
            "topic", 10,
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
        REQUIRE_THROWS(node.create_subscription<std_msgs::msg::String>(
            "topic", 10,
            std::bind(&obelisk::ObeliskNodeTester::GenericCallback<
                          std_msgs::msg::String>,
                      &node, _1)));

        // Try with the override flag
        REQUIRE_NOTHROW(node.create_subscription<std_msgs::msg::String>(
            "topic", 10,
            std::bind(&obelisk::ObeliskNodeTester::GenericCallback<
                          std_msgs::msg::String>,
                      &node, _1),
            true));
    }

    rclcpp::shutdown();
}

TEST_CASE("Subscriber from string", "[obelisk_node]") {
    rclcpp::init(0, nullptr);

    obelisk::ObeliskNodeTester node;

    SECTION("Subscribers") {
        // Check with good config string
        REQUIRE_NOTHROW(node.SubscriptionConfigStrTester<
                        obelisk_control_msgs::msg::PositionSetpoint>(
            "topic:test1,depth:10"));

        REQUIRE_NOTHROW(node.SubscriptionConfigStrTester<
                        obelisk_control_msgs::msg::PositionSetpoint>(
            "topic:test1,depth:10,non_obelisk:true"));

        // Check without topic
        REQUIRE_THROWS(
            node.SubscriptionConfigStrTester<
                obelisk_control_msgs::msg::PositionSetpoint>("depth:10"));

        // Check when missing a colon
        REQUIRE_THROWS(node.SubscriptionConfigStrTester<
                       obelisk_control_msgs::msg::PositionSetpoint>(
            "topic:test1,depth 10"));
    }

    SECTION("Publishers") {
        // Check with good config string
        REQUIRE_NOTHROW(node.PublishConfigStrTester<
                        obelisk_control_msgs::msg::PositionSetpoint>(
            "topic:test1,depth:10"));

        REQUIRE_NOTHROW(node.PublishConfigStrTester<
                        obelisk_control_msgs::msg::PositionSetpoint>(
            "topic:test1,depth:10,non_obelisk:true"));

        // Check without topic
        REQUIRE_THROWS(
            node.PublishConfigStrTester<
                obelisk_control_msgs::msg::PositionSetpoint>("depth:10"));

        // Check when missing a colon
        REQUIRE_THROWS(node.PublishConfigStrTester<
                       obelisk_control_msgs::msg::PositionSetpoint>(
            "topic:test1,depth 10"));
    }

    SECTION("Timers") {
        // Check with good config string
        REQUIRE_NOTHROW(node.TimerConfigFromStrTester("timer_period_sec:2"));

        REQUIRE_NOTHROW(node.TimerConfigFromStrTester("timer_period_sec:.5"));

        // Check without period
        REQUIRE_THROWS(node.TimerConfigFromStrTester("depth:10"));

        // Check when missing a colon
        REQUIRE_THROWS(
            node.TimerConfigFromStrTester("timer_period_sec:2, test"));
    }

    rclcpp::shutdown();
}

// -------------------------------------------------------- //
// --------------- Obelisk Controller Tests --------------- //
// -------------------------------------------------------- //
TEST_CASE("Obelisk Controller Default Pub/Sub",
          "[obelisk_node][obelisk_controller]") {
    rclcpp::init(0, nullptr);

    obelisk::ObeliskControllerTester node;

    // SECTION("Create ")

    rclcpp::shutdown();
}
