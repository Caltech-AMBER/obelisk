#include "obelisk_ros_utils.h"
#include "position_setpoint_controller.h"

int main(int argc, char* argv[]) {
    obelisk::utils::SpinObelisk<PositionSetpointController, rclcpp::executors::MultiThreadedExecutor>(
        argc, argv, "position_setpoint_controller");
}
