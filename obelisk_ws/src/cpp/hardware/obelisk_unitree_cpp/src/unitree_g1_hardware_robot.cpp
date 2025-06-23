#include "rclcpp/rclcpp.hpp"

#include "g1_interface.h"
#include "obelisk_ros_utils.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

int main(int argc, char* argv[]) {
    obelisk::utils::SpinObelisk<obelisk::G1Interface, rclcpp::executors::SingleThreadedExecutor>(argc, argv,
                                                                                                 "unitree_robot");
}
