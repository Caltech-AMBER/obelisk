#include <catch2/catch_test_macros.hpp>

#include "obelisk_robot.h"

namespace obelisk {
    class ObeliskRobotTester : public ObeliskRobot<obelisk_control_msgs::msg::PositionSetpoint> {
      public:
        ObeliskRobotTester() : ObeliskRobot("obelisk_sensor_tester") {
            this->set_parameter(rclcpp::Parameter("sub_ctrl_setting", "topic:topic4"));
            this->set_parameter(rclcpp::Parameter("callback_group_setting", ""));
        }

        void Configure() {
            CHECK(this->on_configure(this->get_current_state()) ==
                  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
            CHECK(this->GetSubscription<obelisk_control_msgs::msg::PositionSetpoint>(this->sub_ctrl_key_) != nullptr);
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
        void ApplyControl(const obelisk_control_msgs::msg::PositionSetpoint& msg) {}
    };
} // namespace obelisk

TEST_CASE("Obelisk Robot Basic Tests", "[obelisk_robot]") {
    rclcpp::init(0, nullptr);

    obelisk::ObeliskRobotTester node;
    node.Configure();
    node.Activate();
    node.Deactivate();
    node.Shutdown();
    node.Cleanup();

    rclcpp::shutdown();
}
