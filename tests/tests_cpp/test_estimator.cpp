#include <catch2/catch_test_macros.hpp>

#include "obelisk_estimator.h"

namespace obelisk {
    class ObeliskEstimatorTester : public ObeliskEstimator<obelisk_estimator_msgs::msg::EstimatedState> {
      public:
        ObeliskEstimatorTester() : ObeliskEstimator("obelisk_estimator_tester") {
            this->set_parameter(rclcpp::Parameter("timer_est_setting", "timer_period_sec:1"));
            this->set_parameter(rclcpp::Parameter("pub_est_setting", "topic:topic1"));
            // this->set_parameter(
            //     rclcpp::Parameter("sub_sensor_settings", std::vector<std::string>{"topic:topic2", "topic:topic3"}));
            this->set_parameter(rclcpp::Parameter("callback_group_setting", ""));
        }

        void Configure() {
            REQUIRE(this->estimator_publisher_ == nullptr);
            REQUIRE(this->estimator_timer_ == nullptr);

            REQUIRE(this->on_configure(this->get_current_state()) ==
                    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

            REQUIRE(this->estimator_publisher_ != nullptr);
            REQUIRE(this->estimator_timer_ != nullptr);
        }

        void Activate() {
            REQUIRE(this->on_activate(this->get_current_state()) ==
                    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
        }

        void Deactivate() {
            REQUIRE(this->on_deactivate(this->get_current_state()) ==
                    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
        }

        void Shutdown() {
            REQUIRE(this->on_shutdown(this->get_current_state()) ==
                    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
        }

        void Cleanup() {
            REQUIRE(this->on_cleanup(this->get_current_state()) ==
                    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
        }

      protected:
        obelisk_estimator_msgs::msg::EstimatedState ComputeStateEstimate() override {
            const obelisk_estimator_msgs::msg::EstimatedState msg;
            return msg;
        }
    };
} // namespace obelisk

TEST_CASE("Obelisk Estimator Basic Tests", "[obelisk_estimator]") {
    rclcpp::init(0, nullptr);

    obelisk::ObeliskEstimatorTester node;
    node.Configure();
    node.Activate();
    node.Deactivate();
    node.Shutdown();
    node.Cleanup();

    rclcpp::shutdown();
}