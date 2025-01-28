#include "obelisk_controller.h"
#include "obelisk_ros_utils.h"
#include "sensor_msgs/msg/joy.hpp"
#include "obelisk_control_msgs/msg/execution_fsm.hpp"
#include "obelisk_control_msgs/msg/velocity_command.hpp"

namespace obelisk {
    using unitree_fsm_msg = obelisk_control_msgs::msg::ExecutionFSM;
    using unitree_high_level_ctrl_msg = obelisk_control_msgs::msg::VelocityCommand;

    class UnitreeJoystick : public ObeliskController<unitree_high_level_ctrl_msg, sensor_msgs::msg::Joy> {
      public:
        UnitreeJoystick(const std::string& name)
            : ObeliskController<unitree_high_level_ctrl_msg, sensor_msgs::msg::Joy>(name) {

            this->declare_parameter<float>("v_x_max", 0.5);
            this->declare_parameter<float>("v_y_max", 0.5);
            this->declare_parameter<float>("w_z_max", 0.5);
            v_x_max_ = this->get_parameter("v_x_max").as_double();
            v_y_max_ = this->get_parameter("v_y_max").as_double();
            w_z_max_ = this->get_parameter("w_z_max").as_double();

            // Register publishers
            this->RegisterObkPublisher<unitree_fsm_msg>("pub_exec_fsm_setting", pub_exec_fsm_key_);

            RCLCPP_INFO_STREAM(this->get_logger(), "UnitreeJoystick node has been initialized.");
        }

      protected:
        void UpdateXHat(__attribute__((unused)) const sensor_msgs::msg::Joy& msg) override {
            // Update velocity commands
            constexpr int LEFT_STICK_X = 0;
            constexpr int LEFT_STICK_Y = 1;
            constexpr int RIGHT_STICK_X = 3;

            v_x_ = msg.axes[LEFT_STICK_Y];
            v_y_ = msg.axes[LEFT_STICK_X];
            w_z_ = msg.axes[RIGHT_STICK_X];

            // ----- Axes ----- //
            // TODO: Check these
            constexpr int DPAD_VERTICAL = 7;
            constexpr int DPAD_HORIZONTAL = 6;

            // Print control menu
            constexpr int MENU = 7;
            static rclcpp::Time last_menu_press = this->now();
            if ((this->now() - last_menu_press).seconds() > 1 && msg.buttons[MENU]) {
                RCLCPP_INFO_STREAM(this->get_logger(),
                    "UnitreeJoystick Button Layout:\n" <<
                    "Execution Finite State Machine:\n" << 
                    "\tHOME:            DPAD_LEFT\n" <<
                    "\tDAMPING:         DPAD_RIGHT\n" <<
                    "\tLOW_LEVEL_CTRL:  DPAD_DOWN\n" <<
                    "\tHIGH_LEVEL_CTRL: DPAD_DOWN\n" <<
                    "High Level Velocity Commands:\n" <<
                    "\tFront/Back: Left Stick Vertical\n" <<
                    "\tSide/Side:  Left Stick Horizontal\n" <<
                    "\tYaw Rate:   Right Stick Horizontal"
                );
                last_menu_press = this->now();
            }

            // DPAD for FSM
            static rclcpp::Time last_Dpad_press = this->now();
            if ((this->now() - last_Dpad_press).seconds() > 1) {
                unitree_fsm_msg fsm_msg;
                fsm_msg.header.stamp = this->now();

                if (msg.axes[DPAD_HORIZONTAL] < -0.5) {
                    fsm_msg.cmd_exec_fsm_state = 4;         // Damping
                } else if (msg.axes[DPAD_HORIZONTAL] > 0.5) {
                    fsm_msg.cmd_exec_fsm_state = 1;         // Home
                } else if (msg.axes[DPAD_VERTICAL] > 0.5) {
                    fsm_msg.cmd_exec_fsm_state = 3;         // High Level Contrl
                } else if (msg.axes[DPAD_VERTICAL] < -0.5) {
                    fsm_msg.cmd_exec_fsm_state = 2;         // Low Level Control
                } else {
                    return;
                }
                last_Dpad_press = this->now();
                this->GetPublisher<unitree_fsm_msg>(pub_exec_fsm_key_)->publish(fsm_msg);
                RCLCPP_INFO_STREAM(this->get_logger(), "UnitreeJoystick sent Execution FSM Command.");
            }
        }

        unitree_high_level_ctrl_msg ComputeControl() override {
            unitree_high_level_ctrl_msg msg;
            msg.v_x = v_x_ * v_x_max_;
            msg.v_y = v_y_ * v_y_max_;
            msg.w_z = w_z_ * w_z_max_;

            this->GetPublisher<unitree_high_level_ctrl_msg>(this->ctrl_key_)->publish(msg);

            return msg;
        };
    
        const std::string pub_exec_fsm_key_ = "pub_exec_fsm_key";
        float v_x_ = 0;
        float v_y_ = 0; 
        float w_z_ = 0;
        float v_x_max_;
        float v_y_max_; 
        float w_z_max_;
      private:
    };
}