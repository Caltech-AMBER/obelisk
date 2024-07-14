#include <catch2/catch_test_macros.hpp>

#include "obelisk_viz_robot_default.h"

TEST_CASE("Obelisk Viz Robot Basic Tests", "[obelisk_viz_robot_default]") {
    rclcpp::init(0, nullptr);
    obelisk::viz::ObeliskVizRobotDefault<obelisk_estimator_msgs::msg::EstimatedState> viz_robot("default_tester");

    // viz_robot.Configure();
    // viz_robot.Activate();
    // viz_robot.Deactivate();
    // viz_robot.Shutdown();
    // viz_robot.Cleanup();

    rclcpp::shutdown();
}
