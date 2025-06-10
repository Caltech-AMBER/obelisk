#include "rclcpp/rclcpp.hpp"

#include "obelisk_ros_utils.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "unitree_joystick.h"

int main(int argc, char* argv[]) {
    obelisk::utils::SpinObelisk<obelisk::UnitreeJoystick, rclcpp::executors::SingleThreadedExecutor>(
        argc, argv, "unitree_joystick");
}
