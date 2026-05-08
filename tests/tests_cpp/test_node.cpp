#include <catch2/catch_test_macros.hpp>

#include "obelisk_node.h"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

namespace obelisk {
    /**
     * Minimal subclass that lets us register components and exercise the new YAML-driven flow.
     */
    class ObeliskNodeTester : public ObeliskNode {
      public:
        ObeliskNodeTester() : ObeliskNode("obelisk_tester") {}

        // expose for tests
        using ObeliskNode::create_publisher;
        using ObeliskNode::create_subscription;

        template <typename MessageT> void GenericCallback(const MessageT& msg) {}
    };
} // namespace obelisk

// -------------------------------------------------- //
// --------------- Obelisk Node Tests --------------- //
// -------------------------------------------------- //
TEST_CASE("Obelisk Node Pub and Sub", "[obelisk_node]") {
    rclcpp::init(0, nullptr);

    obelisk::ObeliskNodeTester node;

    SECTION("Direct publishers") {
        REQUIRE_NOTHROW(node.create_publisher<std_msgs::msg::String>("topic", 10));
        REQUIRE_NOTHROW(node.create_publisher<obelisk_control_msgs::msg::PositionSetpoint>("topic", 10));
        REQUIRE_NOTHROW(node.create_publisher<obelisk_estimator_msgs::msg::EstimatedState>("topic", 10));
        REQUIRE_NOTHROW(node.create_publisher<obelisk_sensor_msgs::msg::ObkJointEncoders>("topic", 10));
        REQUIRE_NOTHROW(node.create_publisher<obelisk_sensor_msgs::msg::TrueSimState>("topic", 10));
    }

    SECTION("Direct subscribers") {
        REQUIRE_NOTHROW(node.create_subscription<obelisk_control_msgs::msg::PositionSetpoint>(
            "topic", 10,
            std::bind(&obelisk::ObeliskNodeTester::GenericCallback<obelisk_control_msgs::msg::PositionSetpoint>, &node,
                      _1)));
        REQUIRE_NOTHROW(node.create_subscription<std_msgs::msg::String>(
            "topic", 10, std::bind(&obelisk::ObeliskNodeTester::GenericCallback<std_msgs::msg::String>, &node, _1)));
    }

    rclcpp::shutdown();
}

TEST_CASE("Obelisk Node configures pub/sub/timer from obelisk_settings YAML", "[obelisk_node]") {
    rclcpp::init(0, nullptr);

    obelisk::ObeliskNodeTester node;

    // Register a publisher, subscription, and timer by key.
    node.RegisterObkPublisher<obelisk_control_msgs::msg::PositionSetpoint>("pub_x");
    node.RegisterObkSubscription<obelisk_control_msgs::msg::PositionSetpoint>(
        "sub_x", std::bind(&obelisk::ObeliskNodeTester::GenericCallback<obelisk_control_msgs::msg::PositionSetpoint>,
                           &node, _1));
    bool fired = false;
    node.RegisterObkTimer("timer_x", [&fired]() { fired = true; });

    const std::string settings = R"(
publishers:
  - key: pub_x
    topic: /test/pub
    history_depth: 7
subscribers:
  - key: sub_x
    topic: /test/sub
    history_depth: 3
timers:
  - key: timer_x
    timer_period_sec: 0.01
)";
    node.set_parameter(rclcpp::Parameter("obelisk_settings", settings));

    REQUIRE(node.on_configure(node.get_current_state()) ==
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

    auto pub = node.GetPublisher<obelisk_control_msgs::msg::PositionSetpoint>("pub_x");
    REQUIRE(pub != nullptr);
    CHECK(pub->get_actual_qos().depth() == 7);

    auto sub = node.GetSubscription<obelisk_control_msgs::msg::PositionSetpoint>("sub_x");
    REQUIRE(sub != nullptr);
    CHECK(sub->get_actual_qos().depth() == 3);

    REQUIRE(node.GetTimer("timer_x") != nullptr);

    rclcpp::shutdown();
}

TEST_CASE("Empty obelisk_settings is a no-op (no components registered)", "[obelisk_node]") {
    rclcpp::init(0, nullptr);

    obelisk::ObeliskNodeTester node;
    // Default value of obelisk_settings is "" (empty string).
    REQUIRE(node.on_configure(node.get_current_state()) ==
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

    rclcpp::shutdown();
}

TEST_CASE("Malformed YAML in obelisk_settings raises", "[obelisk_node]") {
    rclcpp::init(0, nullptr);

    obelisk::ObeliskNodeTester node;
    node.set_parameter(rclcpp::Parameter("obelisk_settings", std::string("publishers: [unterminated")));
    REQUIRE_THROWS(node.on_configure(node.get_current_state()));

    rclcpp::shutdown();
}
