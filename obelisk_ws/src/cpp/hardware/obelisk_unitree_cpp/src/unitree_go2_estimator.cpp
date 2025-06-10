#include "unitree_go2_estimator.h"
#include "obelisk_ros_utils.h"

int main(int argc, char* argv[]) {
    obelisk::utils::SpinObelisk<obelisk::UnitreeGo2Estimator, rclcpp::executors::SingleThreadedExecutor>(
        argc, argv, "unitree_go2_estimator");
}
