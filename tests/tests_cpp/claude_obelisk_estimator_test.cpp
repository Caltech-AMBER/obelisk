#include "obelisk_estimator.h"
#include "obelisk_estimator_msgs/msg/estimated_state.hpp"
#include <catch2/catch_test_macros.hpp>

class TestObeliskEstimator : public obelisk::ObeliskEstimator<obelisk_estimator_msgs::msg::EstimatedState> {
  public:
    TestObeliskEstimator() : ObeliskEstimator("test_estimator") {}

    obelisk_estimator_msgs::msg::EstimatedState ComputeStateEstimate() override {
        return obelisk_estimator_msgs::msg::EstimatedState();
    }

    using ObeliskEstimator::estimator_publisher_;
    using ObeliskEstimator::estimator_timer_;
};

TEST_CASE("ObeliskEstimator Construction and Configuration", "[ObeliskEstimator]") {
    rclcpp::init(0, nullptr);

    TestObeliskEstimator estimator;

    estimator.set_parameter(rclcpp::Parameter("timer_est_setting", "timer_period_sec:0.1"));
    estimator.set_parameter(rclcpp::Parameter("pub_est_setting", "topic:/estimated_state,history_depth:10"));

    auto result = estimator.on_configure(rclcpp_lifecycle::State());
    REQUIRE(result == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

    REQUIRE(estimator.estimator_publisher_ != nullptr);
    REQUIRE(estimator.estimator_timer_ != nullptr);
    // REQUIRE(estimator.sub_sensor_config_strs_ == test_configs);

    rclcpp::shutdown();
}

TEST_CASE("ObeliskEstimator Activation and Deactivation", "[ObeliskEstimator]") {
    rclcpp::init(0, nullptr);

    TestObeliskEstimator estimator;
    estimator.set_parameter(rclcpp::Parameter("timer_est_setting", "timer_period_sec:0.1"));
    estimator.set_parameter(rclcpp::Parameter("pub_est_setting", "topic:/estimated_state,history_depth:10"));
    estimator.on_configure(rclcpp_lifecycle::State());

    auto activate_result = estimator.on_activate(rclcpp_lifecycle::State());
    REQUIRE(activate_result == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

    auto deactivate_result = estimator.on_deactivate(rclcpp_lifecycle::State());
    REQUIRE(deactivate_result == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

    rclcpp::shutdown();
}

TEST_CASE("ObeliskEstimator Cleanup and Shutdown", "[ObeliskEstimator]") {
    rclcpp::init(0, nullptr);

    TestObeliskEstimator estimator;
    estimator.set_parameter(rclcpp::Parameter("timer_est_setting", "timer_period_sec:0.1"));
    estimator.set_parameter(rclcpp::Parameter("pub_est_setting", "topic:/estimated_state,history_depth:10"));

    estimator.on_configure(rclcpp_lifecycle::State());

    auto cleanup_result = estimator.on_cleanup(rclcpp_lifecycle::State());
    REQUIRE(cleanup_result == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
    REQUIRE(estimator.estimator_publisher_ == nullptr);
    REQUIRE(estimator.estimator_timer_ == nullptr);

    auto shutdown_result = estimator.on_shutdown(rclcpp_lifecycle::State());
    REQUIRE(shutdown_result == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
    REQUIRE(estimator.estimator_publisher_ == nullptr);
    REQUIRE(estimator.estimator_timer_ == nullptr);

    rclcpp::shutdown();
}

TEST_CASE("ObeliskEstimator with Empty Sensor Settings", "[ObeliskEstimator]") {
    rclcpp::init(0, nullptr);

    TestObeliskEstimator estimator;
    estimator.set_parameter(rclcpp::Parameter("timer_est_setting", "timer_period_sec:0.1"));
    estimator.set_parameter(rclcpp::Parameter("pub_est_setting", "topic:/estimated_state,history_depth:10"));

    auto result = estimator.on_configure(rclcpp_lifecycle::State());
    REQUIRE(result == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

    REQUIRE(estimator.estimator_publisher_ != nullptr);
    REQUIRE(estimator.estimator_timer_ != nullptr);

    rclcpp::shutdown();
}