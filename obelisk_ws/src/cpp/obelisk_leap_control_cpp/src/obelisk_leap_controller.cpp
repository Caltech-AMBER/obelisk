#include "leap_position_setpoint_controller.h"
#include "obelisk_ros_utils.h"

int main(int argc, char* argv[]) {
    obelisk::utils::SpinObelisk<obelisk::LeapPositionSetpointController, rclcpp::executors::SingleThreadedExecutor>(
        argc, argv, "position_setpoint_controller");
}
