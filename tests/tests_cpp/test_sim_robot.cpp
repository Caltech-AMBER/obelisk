#include <catch2/catch_test_macros.hpp>

#include "obelisk_sim_robot.h"

namespace obelisk {
    class ObeliskSimRobotTester : public ObeliskSimRobot<obelisk_control_msgs::msg::PositionSetpoint> {
      public:
        ObeliskSimRobotTester() : ObeliskSimRobot("obelisk_sim_robot_tester") {
            this->set_parameter(rclcpp::Parameter("timer_true_sim_state_setting", "topic:topic3,timer_period_sec:1"));
            this->set_parameter(rclcpp::Parameter("pub_true_sim_state_setting", "topic:topic4"));
            this->set_parameter(rclcpp::Parameter("callback_group_settings", "topic:topic2"));
            this->set_parameter(rclcpp::Parameter("sub_ctrl_setting", "topic:topic5"));
        }

        void Configure() {
            CHECK(this->on_configure(this->get_current_state()) ==
                  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS);
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

        void RunSimulator() override {
            CHECK(this->stop_thread_ == false);
            while (!this->stop_thread_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            CHECK(stop_thread_ == true);
        }

        obelisk_sensor_msgs::msg::TrueSimState PublishTrueSimState() {
            obelisk_sensor_msgs::msg::TrueSimState msg;
            return msg;
        }
    };
} // namespace obelisk

TEST_CASE("Obelisk Sim Robot Basic Tests", "[obelisk_robot]") {
    rclcpp::init(0, nullptr);

    obelisk::ObeliskSimRobotTester node;
    node.Configure();
    node.Activate();
    node.Deactivate();
    node.Shutdown();
    node.Cleanup();

    rclcpp::shutdown();
}
