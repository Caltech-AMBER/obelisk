#include "obelisk_node.h"
#include "sensor_msgs/msg/joy.hpp"
#include "obelisk_control_msgs/msg/execution_fsm.h"
#include "obelisk_control_msgs/msg/velocity_command.h"

namespace obelisk {
    using unitree_fsm_msg = obelisk_control_msgs::msg::ExecutionFSM;
    using unitree_high_level_ctrl_msg = obelisk_control_msgs::msg::VelocityCommand;

    class UnitreeJoystick : public rclcpp::Node {
    public:
        UnitreeJoystick(): Node("unitree_joystick_node") {
            // Subscriber to /obelisk/go2/joy
            this->RegisterObkSubscription<sensor_msgs::msg::Joy>(
                        "joystick_sub_setting", "joystick_sub_key",
                        std::bind(&UnitreeJoystick::JoystickCallback, this, std::placeholders::_1));

            // Register publishers
            this->RegisterObkPublisher<unitree_high_level_ctrl_msg>("pub_high_level_ctrl_setting", "high_level_ctrl_key");
            this->RegisterObkPublisher<unitree_fsm_msg>("pub_exec_fsm_setting", "exec_fsm_key");

            RCLCPP_INFO(this->get_logger(), "UnitreeJoystick node has been initialized.");
        }

    private:
        // Callback for the /obelisk/go2/joy topic
        void JoystickCallback(const sensor_msgs::msg::Joy& msg) {

            unitree_high_level_ctrl_msg vel_cmd;
            vel_cmd.header.stamp = this->now();
            vel_cmd.v_x = msg.axes[1];
            vel_cmd.v_y = -msg.axes[0];
            vel_cmd.w_z = msg.axes[2];
            this->GetPublisher<unitree_high_level_ctrl_msg>("high_level_ctrl_key")->publish(vel_cmd);

            // ----- Axes ----- //
            // TODO: Check these
            constexpr int DPAD_VERTICAL = 7;
            constexpr int DPAD_HORIZONTAL = 6;
            constexpr int DPAD_RIGHT = 8;
            constexpr int DPAD_LEFT = 9;

            static rclcpp::Time last_Dpad_press = this->now();

            if ((this->now() - last_Dpad_press).seconds() > 1 && (msg.buttons[DPAD_RIGHT] || msg.buttons[DPAD_LEFT] || msg.buttons[DPAD_UP] || msg.buttons[DPAD_DOWN])) {
                unitree_fsm_msg fsm_msg;
                fsm_msg.header.stamp = this->now();

                if (msg.buttons[DPAD_RIGHT]) {
                    msg.cmd_exec_fsm_state = 4;         // Damping
                } else if (msg.buttons[DPAD_LEFT]) {
                    msg.cmd_exec_fsm_state = 3;         // Home
                } else if (msg.buttons[DPAD_UP]) {
                    msg.cmd_exec_fsm_state = 2;         // High Level Contrl
                } else if (msg.buttons[DPAD_DOWN]) {
                    msg.cmd_exec_fsm_state = 1;         // Low Level Control
                }
                this->GetPublisher<unitree_high_level_ctrl_msg>("exec_fsm_key")->publish(fsm_msg);
            }
        }
    };
}