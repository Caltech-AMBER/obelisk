#include "obelisk_controller.h"
#include <catch2/catch_test_macros.hpp>

namespace obelisk {
    class ObeliskControllerTester : public ObeliskController<obelisk_control_msgs::msg::PositionSetpoint,
                                                             obelisk_estimator_msgs::msg::EstimatedState> {
      public:
        ObeliskControllerTester() : ObeliskController("obelisk_controller_tester") {
            this->set_parameter(rclcpp::Parameter("timer_ctrl_settings", "timer_period_sec:1"));
            this->set_parameter(rclcpp::Parameter("pub_ctrl_settings", "topic:topic1"));
            this->set_parameter(rclcpp::Parameter("sub_est_settings", "topic:topic2"));
            this->set_parameter(rclcpp::Parameter("callback_group_settings", ""));
        }

        void Configure() {
            REQUIRE(this->control_publisher_ == nullptr);
            REQUIRE(this->control_timer_ == nullptr);
            REQUIRE(this->state_estimator_subscriber_ == nullptr);

            REQUIRE(this->on_configure(this->get_current_state()) ==
                    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);

            REQUIRE(this->control_publisher_ != nullptr);
            REQUIRE(this->control_timer_ != nullptr);
            REQUIRE(this->state_estimator_subscriber_ != nullptr);
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
