#include <catch2/catch_test_macros.hpp>

#include "obelisk_mujoco_sim_robot.h"

namespace obelisk {
    class ObeliskMujocoTester : public ObeliskMujocoRobot<obelisk_control_msgs::msg::PositionSetpoint> {
      public:
        ObeliskMujocoTester() : ObeliskMujocoRobot("obelisk_mujoco_tester") {
            this->set_parameter(rclcpp::Parameter("timer_true_sim_state_setting", "topic:topic1,timer_period_sec:1"));
            this->set_parameter(rclcpp::Parameter("pub_true_sim_state_setting", "topic:topic4"));
            this->set_parameter(rclcpp::Parameter("callback_group_setting", "topic:topic2"));
            this->set_parameter(rclcpp::Parameter(
                "mujoco_setting", "model_xml_path:models/dummy/"
                                  "dummy.xml,n_u:1,sensor_settings:[{group_name=group1|dt=0.01|topic=topic1|sensor_"
                                  "type=jointpos|sensor_names=sensor_joint1&sensor_joint2+group_name=group2|topic="
                                  "topic2|dt=0.5|sensor_type=jointpos|sensor_names=sensor_joint3}]"));
            this->set_parameter(rclcpp::Parameter("sub_ctrl_setting", "topic:topic5"));
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
    };
} // namespace obelisk

TEST_CASE("Obelisk Mujoco Basic Tests", "[obelisk_robot]") {
    rclcpp::init(0, nullptr);

    obelisk::ObeliskMujocoTester node;
    node.Configure();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    node.Activate();
    node.Deactivate();
    node.Shutdown();
    node.Cleanup();

    rclcpp::shutdown();
}
