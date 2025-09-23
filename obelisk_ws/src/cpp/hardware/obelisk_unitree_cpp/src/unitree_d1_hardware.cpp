#include "rclcpp/rclcpp.hpp"

#include "d1_interface.h"
#include "obelisk_ros_utils.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

int main(int argc, char* argv[]) {
    obelisk::utils::SpinObelisk<obelisk::D1Interface, rclcpp::executors::SingleThreadedExecutor>(argc, argv, "unitree_robot");
}
