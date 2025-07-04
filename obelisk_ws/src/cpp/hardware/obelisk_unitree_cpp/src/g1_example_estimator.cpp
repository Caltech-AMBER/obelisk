#include "obelisk_ros_utils.h"
#include "unitree_example_estimator.h"

int main(int argc, char* argv[]) {
    obelisk::utils::SpinObelisk<obelisk::UnitreeExampleEstimator, rclcpp::executors::SingleThreadedExecutor>(
        argc, argv, "unitree_example_controller");
}
