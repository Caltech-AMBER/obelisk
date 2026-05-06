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
            const std::string settings = R"(
publishers:
  - key: pub_viz_joint
    topic: topic1
    history_depth: 10
subscribers:
  - key: sub_est
    topic: topic2
    history_depth: 10
timers:
  - key: time_joints
    timer_period_sec: 1.0
)";
            this->set_parameter(rclcpp::Parameter("obelisk_settings", settings));
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
