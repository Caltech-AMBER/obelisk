#include "obelisk_ros_utils.h"
#include "obelisk_viz_robot_default.h"

int main(int argc, char* argv[]) {
    obelisk::utils::SpinObelisk<obelisk::viz::ObeliskVizRobotDefault<obelisk_estimator_msgs::msg::EstimatedState>,
                                rclcpp::executors::MultiThreadedExecutor>(argc, argv, "robot_viz");
}
