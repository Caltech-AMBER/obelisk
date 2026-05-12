#include "rclcpp/rclcpp.hpp"

#include "obelisk_ros_utils.h"
#include "themis_interface.h"

int main(int argc, char* argv[]) {
    obelisk::utils::SpinObelisk<obelisk::ThemisInterface,
                                rclcpp::executors::SingleThreadedExecutor>(argc, argv, "themis_robot");
}
