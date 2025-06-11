#include "rclcpp/rclcpp.hpp"

#include "go2_interface.h"
#include "obelisk_ros_utils.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

int main(int argc, char* argv[]) {
    obelisk::utils::SpinObelisk<obelisk::Go2Interface, rclcpp::executors::SingleThreadedExecutor>(argc, argv,
                                                                                                  "unitree_robot");
}
