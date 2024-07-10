#include "obelisk_estimator.h"
#include "obelisk_estimator_msgs/msg/estimated_state.hpp"
#include <catch2/catch_test_macros.hpp>

class TestObeliskEstimator : public obelisk::ObeliskEstimator<obelisk_estimator_msgs::msg::EstimatedState> {
  public:
    TestObeliskEstimator() : ObeliskEstimator("test_estimator") {}

    obelisk_estimator_msgs::msg::EstimatedState ComputeStateEstimate() override {
        return obelisk_estimator_msgs::msg::EstimatedState();
    }
};

TEST_CASE("ObeliskEstimator Construction and Configuration", "[ObeliskEstimator]") {
    rclcpp::init(0, nullptr);

    TestObeliskEstimator estimator;

    estimator.set_parameter(rclcpp::Parameter("timer_est_setting", "timer_period_sec:0.1"));
    estimator.set_parameter(rclcpp::Parameter("pub_est_setting", "topic:/estimated_state,history_depth:10"));

    auto result = estimator.on_configure(rclcpp_lifecycle::State());
    CHECK(result == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

    rclcpp::shutdown();
}

TEST_CASE("ObeliskEstimator Activation and Deactivation", "[ObeliskEstimator]") {
    rclcpp::init(0, nullptr);

    TestObeliskEstimator estimator;
    estimator.set_parameter(rclcpp::Parameter("timer_est_setting", "timer_period_sec:0.1"));
    estimator.set_parameter(rclcpp::Parameter("pub_est_setting", "topic:/estimated_state,history_depth:10"));
    estimator.on_configure(rclcpp_lifecycle::State());

    auto activate_result = estimator.on_activate(rclcpp_lifecycle::State());
    CHECK(activate_result == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

    auto deactivate_result = estimator.on_deactivate(rclcpp_lifecycle::State());
    CHECK(deactivate_result == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

    rclcpp::shutdown();
}

TEST_CASE("ObeliskEstimator Cleanup and Shutdown", "[ObeliskEstimator]") {
    rclcpp::init(0, nullptr);

    TestObeliskEstimator estimator;
    estimator.set_parameter(rclcpp::Parameter("timer_est_setting", "timer_period_sec:0.1"));
    estimator.set_parameter(rclcpp::Parameter("pub_est_setting", "topic:/estimated_state,history_depth:10"));

    estimator.on_configure(rclcpp_lifecycle::State());

    auto cleanup_result = estimator.on_cleanup(rclcpp_lifecycle::State());
    CHECK(cleanup_result == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

    auto shutdown_result = estimator.on_shutdown(rclcpp_lifecycle::State());
    CHECK(shutdown_result == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

    rclcpp::shutdown();
}

TEST_CASE("ObeliskEstimator with Empty Sensor Settings", "[ObeliskEstimator]") {
    rclcpp::init(0, nullptr);

    TestObeliskEstimator estimator;
    estimator.set_parameter(rclcpp::Parameter("timer_est_setting", "timer_period_sec:0.1"));
    estimator.set_parameter(rclcpp::Parameter("pub_est_setting", "topic:/estimated_state,history_depth:10"));

    auto result = estimator.on_configure(rclcpp_lifecycle::State());
    CHECK(result == rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

    rclcpp::shutdown();
}
