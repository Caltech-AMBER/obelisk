#include "rclcpp/rclcpp.hpp"

#include "leap_hand_interface.h"
#include "obelisk_ros_utils.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

int main(int argc, char* argv[]) {
    obelisk::utils::SpinObelisk<obelisk::ObeliskLeapHand, rclcpp::executors::MultiThreadedExecutor>(argc, argv,
                                                                                                    "leap_hand");
}
