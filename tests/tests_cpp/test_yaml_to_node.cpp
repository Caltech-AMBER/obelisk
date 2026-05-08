// Behavioral safety-net tests for the C++ side of the YAML-config -> ROS-parameter -> running-node pipeline.
//
// These tests pin down the *observable* properties of an Obelisk node configured from the single
// ``obelisk_settings`` ROS parameter (a YAML string): topic names, history depth (QoS), and msg-type
// wiring. They mirror the safety-net Python tests in tests/tests_python/tests_core/test_yaml_to_node.py.

#include <catch2/catch_test_macros.hpp>

#include "obelisk_controller.h"
#include "obelisk_estimator.h"

namespace obelisk {

    class SafetyNetController
        : public ObeliskController<obelisk_control_msgs::msg::PositionSetpoint, obelisk_estimator_msgs::msg::EstimatedState> {
      public:
        SafetyNetController() : ObeliskController("safety_net_controller") {}

        void ApplySettings(const std::string& yaml_settings) {
            this->set_parameter(rclcpp::Parameter("obelisk_settings", yaml_settings));
        }

        // expose protected key fields for assertions
        using ObeliskController::ctrl_key_;
        using ObeliskController::est_key_;
        using ObeliskController::timer_key_;

      protected:
        void UpdateXHat(const obelisk_estimator_msgs::msg::EstimatedState&) override {}
        obelisk_control_msgs::msg::PositionSetpoint ComputeControl() override {
            return obelisk_control_msgs::msg::PositionSetpoint{};
        }
    };

    class SafetyNetEstimator : public ObeliskEstimator<obelisk_estimator_msgs::msg::EstimatedState> {
      public:
        SafetyNetEstimator() : ObeliskEstimator("safety_net_estimator") {}

        void ApplySettings(const std::string& yaml_settings) {
            this->set_parameter(rclcpp::Parameter("obelisk_settings", yaml_settings));
        }

        using ObeliskEstimator::est_pub_key_;
        using ObeliskEstimator::est_timer_key_;

      protected:
        obelisk_estimator_msgs::msg::EstimatedState ComputeStateEstimate() override {
            return obelisk_estimator_msgs::msg::EstimatedState{};
        }
    };

} // namespace obelisk

// ------------------------------------------------------------------ //
// Tests                                                              //
// ------------------------------------------------------------------ //

TEST_CASE("Controller YAML-derived params produce the configured pub/sub/timer", "[safety_net][controller]") {
    rclcpp::init(0, nullptr);

    obelisk::SafetyNetController node;
    node.ApplySettings(R"(
publishers:
  - key: pub_ctrl
    topic: /safety_net/ctrl
    history_depth: 7
subscribers:
  - key: sub_est
    topic: /safety_net/est
    history_depth: 3
timers:
  - key: timer_ctrl
    timer_period_sec: 0.005
)");

    REQUIRE(node.on_configure(node.get_current_state()) ==
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

    auto pub = node.GetPublisher<obelisk_control_msgs::msg::PositionSetpoint>(node.ctrl_key_);
    REQUIRE(pub != nullptr);
    CHECK(std::string(pub->get_topic_name()).find("/safety_net/ctrl") != std::string::npos);
    CHECK(pub->get_actual_qos().depth() == 7);

    auto sub = node.GetSubscription<obelisk_estimator_msgs::msg::EstimatedState>(node.est_key_);
    REQUIRE(sub != nullptr);
    CHECK(std::string(sub->get_topic_name()).find("/safety_net/est") != std::string::npos);
    CHECK(sub->get_actual_qos().depth() == 3);

    auto timer = node.GetTimer(node.timer_key_);
    REQUIRE(timer != nullptr);

    rclcpp::shutdown();
}

TEST_CASE("Estimator YAML-derived params expose configured publisher topic + depth", "[safety_net][estimator]") {
    rclcpp::init(0, nullptr);

    obelisk::SafetyNetEstimator node;
    node.ApplySettings(R"(
publishers:
  - key: pub_est
    topic: /safety_net/est_out
    history_depth: 5
timers:
  - key: timer_est
    timer_period_sec: 0.02
)");

    REQUIRE(node.on_configure(node.get_current_state()) ==
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

    auto pub = node.GetPublisher<obelisk_estimator_msgs::msg::EstimatedState>(node.est_pub_key_);
    REQUIRE(pub != nullptr);
    CHECK(std::string(pub->get_topic_name()).find("/safety_net/est_out") != std::string::npos);
    CHECK(pub->get_actual_qos().depth() == 5);

    auto timer = node.GetTimer(node.est_timer_key_);
    REQUIRE(timer != nullptr);

    rclcpp::shutdown();
}
