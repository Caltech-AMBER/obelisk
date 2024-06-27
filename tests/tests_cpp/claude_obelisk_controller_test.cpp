// test_obelisk_controller.cpp

#include "obelisk_control_msgs/msg/position_setpoint.hpp"
#include "obelisk_controller.h"
#include "obelisk_estimator_msgs/msg/estimated_state.hpp"
#include <catch2/catch_test_macros.hpp>

class TestObeliskController : public obelisk::ObeliskController<obelisk_control_msgs::msg::PositionSetpoint,
                                                                obelisk_estimator_msgs::msg::EstimatedState> {
  public:
    TestObeliskController() : ObeliskController("test_controller") {}

    obelisk_control_msgs::msg::PositionSetpoint ComputeControl() override {
        return obelisk_control_msgs::msg::PositionSetpoint();
    }

    void UpdateXHat(const obelisk_estimator_msgs::msg::EstimatedState& msg) override {}

    // Expose protected members for testing
    using ObeliskController::control_publisher_;
    using ObeliskController::control_timer_;
    using ObeliskController::state_estimator_subscriber_;
};

TEST_CASE("ObeliskController Configuration", "[ObeliskController]") {
    rclcpp::init(0, nullptr);

    TestObeliskController controller;

    // Set up parameters
    controller.set_parameter(rclcpp::Parameter("timer_ctrl_settings", "timer_period_sec:0.1"));
    controller.set_parameter(rclcpp::Parameter("pub_ctrl_settings", "topic:/control,history_depth:10"));
    controller.set_parameter(rclcpp::Parameter("sub_est_settings", "topic:/estimate,history_depth:10"));

    // Configure the controller
    auto result = controller.on_configure(rclcpp_lifecycle::State());

    REQUIRE(result == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
    REQUIRE(controller.control_publisher_ != nullptr);
    REQUIRE(controller.state_estimator_subscriber_ != nullptr);
    REQUIRE(controller.control_timer_ != nullptr);

    rclcpp::shutdown();
}

TEST_CASE("ObeliskController Activation", "[ObeliskController]") {
    rclcpp::init(0, nullptr);
    TestObeliskController controller;

    // Configure the controller first
    controller.set_parameter(rclcpp::Parameter("timer_ctrl_settings", "timer_period_sec:0.1"));
    controller.set_parameter(rclcpp::Parameter("pub_ctrl_settings", "topic:/control,history_depth:10"));
    controller.set_parameter(rclcpp::Parameter("sub_est_settings", "topic:/estimate,history_depth:10"));
    controller.on_configure(rclcpp_lifecycle::State());

    // Activate the controller
    auto result = controller.on_activate(rclcpp_lifecycle::State());

    REQUIRE(result == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
    rclcpp::shutdown();
}

TEST_CASE("ObeliskController Deactivation", "[ObeliskController]") {
    rclcpp::init(0, nullptr);
    TestObeliskController controller;

    // Configure and activate the controller first
    controller.set_parameter(rclcpp::Parameter("timer_ctrl_settings", "timer_period_sec:0.1"));
    controller.set_parameter(rclcpp::Parameter("pub_ctrl_settings", "topic:/control,history_depth:10"));
    controller.set_parameter(rclcpp::Parameter("sub_est_settings", "topic:/estimate,history_depth:10"));
    controller.on_configure(rclcpp_lifecycle::State());
    controller.on_activate(rclcpp_lifecycle::State());

    // Deactivate the controller
    auto result = controller.on_deactivate(rclcpp_lifecycle::State());

    REQUIRE(result == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
    rclcpp::shutdown();
}

TEST_CASE("ObeliskController Cleanup", "[ObeliskController]") {
    rclcpp::init(0, nullptr);
    TestObeliskController controller;

    // Configure the controller first
    controller.set_parameter(rclcpp::Parameter("timer_ctrl_settings", "timer_period_sec:0.1"));
    controller.set_parameter(rclcpp::Parameter("pub_ctrl_settings", "topic:/control,history_depth:10"));
    controller.set_parameter(rclcpp::Parameter("sub_est_settings", "topic:/estimate,history_depth:10"));
    controller.on_configure(rclcpp_lifecycle::State());

    // Clean up the controller
    auto result = controller.on_cleanup(rclcpp_lifecycle::State());

    REQUIRE(result == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
    REQUIRE(controller.control_publisher_ == nullptr);
    REQUIRE(controller.state_estimator_subscriber_ == nullptr);
    REQUIRE(controller.control_timer_ == nullptr);
    rclcpp::shutdown();
}

TEST_CASE("ObeliskController Shutdown", "[ObeliskController]") {
    rclcpp::init(0, nullptr);
    TestObeliskController controller;

    // Configure the controller first
    controller.set_parameter(rclcpp::Parameter("timer_ctrl_settings", "timer_period_sec:0.1"));
    controller.set_parameter(rclcpp::Parameter("pub_ctrl_settings", "topic:/control,history_depth:10"));
    controller.set_parameter(rclcpp::Parameter("sub_est_settings", "topic:/estimate,history_depth:10"));
    controller.on_configure(rclcpp_lifecycle::State());

    // Shut down the controller
    auto result = controller.on_shutdown(rclcpp_lifecycle::State());

    REQUIRE(result == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
    REQUIRE(controller.control_publisher_ == nullptr);
    REQUIRE(controller.state_estimator_subscriber_ == nullptr);
    REQUIRE(controller.control_timer_ == nullptr);
    rclcpp::shutdown();
}
