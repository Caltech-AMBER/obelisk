#include <catch2/catch_test_macros.hpp>

#include "obelisk_viz_robot.h"

namespace obelisk::viz {
    class ObeliskVizRobotTester : public ObeliskVizRobot<obelisk_estimator_msgs::msg::EstimatedState> {
      public:
        ObeliskVizRobotTester() : ObeliskVizRobot("obelisk_viz_robot_tester") {
            this->set_parameter(rclcpp::Parameter(
                this->urdf_path_param_,
                "/home/zolkin/AmberLab/Project-Obelisk/obelisk/obelisk_ws/src/robots/g1_description/urdf/g1.urdf"));
            this->set_parameter(rclcpp::Parameter(this->pub_settings_, "topic:topic1"));
            this->set_parameter(rclcpp::Parameter(this->sub_settings_, "topic:topic2"));
            this->set_parameter(rclcpp::Parameter(this->timer_settings_, "timer_period_sec:1"));
            this->set_parameter(rclcpp::Parameter("callback_group_setting", ""));
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
        void ParseEstimatedState(const obelisk_estimator_msgs::msg::EstimatedState& msg) {}
    };
} // namespace obelisk::viz

TEST_CASE("Obelisk Viz Robot Basic Tests", "[obelisk_viz_robot]") {
    rclcpp::init(0, nullptr);
    obelisk::viz::ObeliskVizRobotTester viz_robot;

    viz_robot.Configure();
    viz_robot.Activate();
    viz_robot.Deactivate();
    viz_robot.Shutdown();
    viz_robot.Cleanup();

    rclcpp::shutdown();
}
