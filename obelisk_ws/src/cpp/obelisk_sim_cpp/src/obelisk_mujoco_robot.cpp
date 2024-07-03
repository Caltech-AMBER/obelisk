#include "rclcpp/rclcpp.hpp"
#include <iostream>

#include "obelisk_mujoco_sim_robot.h"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto robot =
        std::make_shared<obelisk::ObeliskMujocoRobot<obelisk_control_msgs::msg::PositionSetpoint>>("mujoco_sim");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(robot->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();
}
