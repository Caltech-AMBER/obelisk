#include "obelisk_ros_utils.h"
#include "unitree_example_controller.h"

int main(int argc, char* argv[]) {
    obelisk::utils::SpinObelisk<obelisk::UnitreeExampleController, rclcpp::executors::SingleThreadedExecutor>(
        argc, argv, "unitree_example_controller");
}
