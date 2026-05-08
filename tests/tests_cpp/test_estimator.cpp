#include <catch2/catch_test_macros.hpp>

#include "obelisk_estimator.h"

namespace obelisk {
    class ObeliskEstimatorTester : public ObeliskEstimator<obelisk_estimator_msgs::msg::EstimatedState> {
      public:
        ObeliskEstimatorTester() : ObeliskEstimator("obelisk_estimator_tester") {
            const std::string settings = R"(
publishers:
  - key: pub_est
    topic: topic1
    history_depth: 10
timers:
  - key: timer_est
    timer_period_sec: 1.0
)";
            this->set_parameter(rclcpp::Parameter("obelisk_settings", settings));
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
