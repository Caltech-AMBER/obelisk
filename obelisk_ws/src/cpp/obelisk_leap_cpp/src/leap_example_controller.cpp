#include "leap_example_position_setpoint_controller.h"
#include "obelisk_ros_utils.h"

int main(int argc, char* argv[]) {
    obelisk::utils::SpinObelisk<obelisk::LeapExamplePositionSetpointController,
                                rclcpp::executors::SingleThreadedExecutor>(argc, argv,
                                                                           "leap_position_setpoint_controller");
}
