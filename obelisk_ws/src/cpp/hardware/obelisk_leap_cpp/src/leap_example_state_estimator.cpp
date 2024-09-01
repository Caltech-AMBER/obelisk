#include "leap_example_state_estimator.h"
#include "obelisk_ros_utils.h"

int main(int argc, char* argv[]) {
    obelisk::utils::SpinObelisk<obelisk::LeapExampleStateEstimator, rclcpp::executors::SingleThreadedExecutor>(
        argc, argv, "leap_example_state_estimator");
}
