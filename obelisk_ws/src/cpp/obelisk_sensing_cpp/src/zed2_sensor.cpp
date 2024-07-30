#include "rclcpp/rclcpp.hpp"

#include "obelisk_ros_utils.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "zed2_sensors.h"

int main(int argc, char* argv[]) {
    obelisk::utils::SpinObelisk<ObeliskZed2Sensors, rclcpp::executors::MultiThreadedExecutor>(argc, argv,
                                                                                              "zed2_sensors");
}
