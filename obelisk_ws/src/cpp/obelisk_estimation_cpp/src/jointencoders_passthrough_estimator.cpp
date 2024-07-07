#include "jointencoders_passthrough_estimator.h"
#include "obelisk_ros_utils.h"

int main(int argc, char* argv[]) {
    obelisk::utils::SpinObelisk<JointEncodersPassthrough, rclcpp::executors::MultiThreadedExecutor>(
        argc, argv, "passthrough_estimator");
}
