#include <catch2/catch_test_macros.hpp>

#include "obelisk_sim_robot.h"

namespace obelisk {
    class ObeliskSimRobotTester : public ObeliskSimRobot<obelisk_control_msgs::msg::PositionSetpoint> {
      public:
        ObeliskSimRobotTester() : ObeliskSimRobot("obelisk_sensor_tester") {
            this->set_parameter(rclcpp::Parameter("timer_true_sim_state_setting", "timer_period_sec:1"));
            this->set_parameter(rclcpp::Parameter("pub_true_sim_state_setting", "topic:topic4"));
            this->set_parameter(rclcpp::Parameter("callback_group_setting", ""));
            this->set_parameter(rclcpp::Parameter("n_u", 2));
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
        void RunSimulator() {
            REQUIRE(stop_thread_ == false);
            while (!stop_thread_) {
                std::cout << "In the thread." << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            REQUIRE(stop_thread_ == true);
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
