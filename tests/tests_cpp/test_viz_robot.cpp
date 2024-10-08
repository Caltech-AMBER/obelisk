#include <catch2/catch_test_macros.hpp>

#include <cstdlib>
#include <filesystem>

#include "obelisk_viz_robot.h"

namespace obelisk::viz {
    class ObeliskVizRobotTester : public ObeliskVizRobot<obelisk_estimator_msgs::msg::EstimatedState> {
      public:
        ObeliskVizRobotTester() : ObeliskVizRobot("obelisk_viz_robot_tester") {
            std::string obk_root            = std::getenv("OBELISK_ROOT");
            std::filesystem::path data_path = obk_root;
            data_path += "/tests/tests_cpp/test_data/r2d2.urdf";
            std::cout << "path: " << data_path << std::endl;
            this->set_parameter(rclcpp::Parameter("urdf_path_param", data_path));
            this->set_parameter(rclcpp::Parameter("pub_viz_joint_setting", "topic:topic1"));
            this->set_parameter(rclcpp::Parameter("sub_viz_est_setting", "topic:topic2"));
            this->set_parameter(rclcpp::Parameter("timer_viz_joint_setting", "timer_period_sec:1"));
            this->set_parameter(rclcpp::Parameter("callback_group_settings", ""));
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
