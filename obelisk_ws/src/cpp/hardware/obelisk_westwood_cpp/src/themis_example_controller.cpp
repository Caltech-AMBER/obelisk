#include "rclcpp/rclcpp.hpp"

#include "obelisk_ros_utils.h"
#include "themis_example_controller.h"

int main(int argc, char* argv[]) {
    obelisk::utils::SpinObelisk<obelisk::ThemisExampleController,
                                rclcpp::executors::SingleThreadedExecutor>(argc, argv, "themis_example_controller");
}
