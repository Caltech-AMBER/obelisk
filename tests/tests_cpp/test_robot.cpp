#include <catch2/catch_test_macros.hpp>

#include "obelisk_robot.h"

namespace obelisk {
    class ObeliskRobotTester : public ObeliskRobot<obelisk_control_msgs::msg::PositionSetpoint> {
      public:
        ObeliskRobotTester() : ObeliskRobot("obelisk_sensor_tester") {
            this->set_parameter(
                rclcpp::Parameter("pub_sensor_settings", std::vector<std::string>{"topic:topic2", "topic:topic3"}));
            this->set_parameter(rclcpp::Parameter("sub_ctrl_settings", "topic:topic4"));
            this->set_parameter(rclcpp::Parameter("callback_group_settings", ""));
        }

        void Configure() {
            REQUIRE(this->on_configure(this->get_current_state()) ==
                    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
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
