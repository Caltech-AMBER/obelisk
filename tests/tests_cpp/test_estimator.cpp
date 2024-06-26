#include "obelisk_estimator.h"
#include <catch2/catch_test_macros.hpp>

namespace obelisk {
    class ObeliskEstimatorTestser : public ObeliskEstimator<obelisk_estimator_msgs::msg::EstimatedState> {
      public:
        ObeliskEstimatorTestser() : ObeliskEstimator("obelisk_controller_tester") {
            this->set_parameter(rclcpp::Parameter("timer_est_config_str", "timer_period_sec:1"));
            this->set_parameter(rclcpp::Parameter("pub_est_config_str", "topic:topic1"));
            this->set_parameter(rclcpp::Parameter("sub_sensor_config_strs", "topic:topic2"));
            this->set_parameter(rclcpp::Parameter("callback_group_config_strs", ""));
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

    obelisk::ObeliskEstimatorTestser node;
    node.Configure();
    node.Activate();
    node.Deactivate();
    node.Shutdown();
    node.Cleanup();

    rclcpp::shutdown();
}
