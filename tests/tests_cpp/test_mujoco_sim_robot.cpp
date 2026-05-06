#include <catch2/catch_test_macros.hpp>

#include "obelisk_mujoco_sim_robot.h"

namespace obelisk {
    class ObeliskMujocoTester : public ObeliskMujocoRobot<obelisk_control_msgs::msg::PositionSetpoint> {
      public:
        ObeliskMujocoTester() : ObeliskMujocoRobot("obelisk_mujoco_tester") {
            const std::string settings = R"(
publishers:
  - key: pub_true_sim_state
    topic: topic4
    history_depth: 10
subscribers:
  - key: sub_ctrl
    topic: topic5
    history_depth: 10
timers:
  - key: timer_true_sim_state
    timer_period_sec: 1.0
)";
            this->set_parameter(rclcpp::Parameter("obelisk_settings", settings));

            const std::string mujoco = R"(
model_xml_path: dummy/dummy.xml
n_u: 1
sensor_settings:
  - topic: topic1
    dt: 0.01
    msg_type: ObkJointEncoders
    sensor_names:
      sensor_joint1: jointpos
      sensor_joint2: jointpos
  - topic: topic2
    dt: 0.5
    msg_type: ObkJointEncoders
    sensor_names:
      sensor_joint3: jointpos
)";
            this->set_parameter(rclcpp::Parameter("mujoco_setting", mujoco));
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
    };
} // namespace obelisk

TEST_CASE("Obelisk Mujoco Basic Tests", "[obelisk_robot]") {
    rclcpp::init(0, nullptr);

    obelisk::ObeliskMujocoTester node;
    // TODO Put back when issue #41 is resolved
    // node.Configure();
    // std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    node.Activate();
    node.Deactivate();
    node.Shutdown();
    node.Cleanup();

    rclcpp::shutdown();
}
