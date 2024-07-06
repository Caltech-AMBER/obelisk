#include <catch2/catch_test_macros.hpp>

#include "obelisk_estimator.h"

namespace obelisk {
    class ObeliskEstimatorTester : public ObeliskEstimator<obelisk_estimator_msgs::msg::EstimatedState> {
      public:
        ObeliskEstimatorTester() : ObeliskEstimator("obelisk_estimator_tester") {
            this->set_parameter(rclcpp::Parameter("timer_est_setting", "timer_period_sec:1"));
            this->set_parameter(rclcpp::Parameter("pub_est_setting", "topic:topic1"));
            this->set_parameter(rclcpp::Parameter("callback_group_setting", ""));
        }

        void Configure() {
            CHECK(this->on_configure(this->get_current_state()) ==
                  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

            CHECK(this->GetPublisher<obelisk_estimator_msgs::msg::EstimatedState>(this->est_pub_key_) != nullptr);
            CHECK(this->GetTimer(this->est_timer_key_) != nullptr);
        }

        void Activate() {
            CHECK(this->on_activate(this->get_current_state()) ==
                  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
        }

        void Deactivate() {
            CHECK(this->on_deactivate(this->get_current_state()) ==
                  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
        }

        void Shutdown() {
            CHECK(this->on_shutdown(this->get_current_state()) ==
                  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
        }

        void Cleanup() {
            CHECK(this->on_cleanup(this->get_current_state()) ==
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
