#include "obelisk_controller.h"
#include <catch2/catch_test_macros.hpp>

namespace obelisk {
    class ObeliskControllerTester : public ObeliskController<obelisk_control_msgs::msg::PositionSetpoint,
                                                             obelisk_estimator_msgs::msg::EstimatedState> {
      public:
        ObeliskControllerTester() : ObeliskController("obelisk_controller_tester") {
            const std::string settings = R"(
publishers:
  - key: pub_ctrl
    topic: topic1
    history_depth: 10
subscribers:
  - key: sub_est
    topic: topic2
    history_depth: 10
timers:
  - key: timer_ctrl
    timer_period_sec: 1.0
)";
            this->set_parameter(rclcpp::Parameter("obelisk_settings", settings));
        }

        void Configure() {
            CHECK(this->on_configure(this->get_current_state()) ==
                  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

            CHECK(this->GetPublisher<obelisk_control_msgs::msg::PositionSetpoint>(this->ctrl_key_) != nullptr);
            CHECK(this->GetTimer(this->timer_key_) != nullptr);
            CHECK(this->GetSubscription<obelisk_estimator_msgs::msg::EstimatedState>(this->est_key_) != nullptr);
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
            CHECK(this->publishers_.empty());
            CHECK(this->subscriptions_.empty());
        }

        void Cleanup() {
            CHECK(this->on_cleanup(this->get_current_state()) ==
                  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
            CHECK(this->publishers_.empty());
            CHECK(this->subscriptions_.empty());
        }

      protected:
        void UpdateXHat(const obelisk_estimator_msgs::msg::EstimatedState& msg) override {}
        obelisk_control_msgs::msg::PositionSetpoint ComputeControl() override {
            obelisk_control_msgs::msg::PositionSetpoint msg;
            return msg;
        };
    };
} // namespace obelisk

// -------------------------------------------------------- //
// --------------- Obelisk Controller Tests --------------- //
// -------------------------------------------------------- //
TEST_CASE("Obelisk Controller Default Pub/Sub", "[obelisk_node][obelisk_controller]") {
    rclcpp::init(0, nullptr);

    obelisk::ObeliskControllerTester node;
    node.Configure();
    node.Activate();
    node.Deactivate();
    node.Shutdown();
    node.Cleanup();

    rclcpp::shutdown();
}
