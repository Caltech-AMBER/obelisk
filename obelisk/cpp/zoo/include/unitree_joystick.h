#include "obelisk_control_msgs/msg/execution_fsm.hpp"
#include "obelisk_control_msgs/msg/velocity_command.hpp"
#include "obelisk_controller.h"
#include "obelisk_ros_utils.h"
#include "sensor_msgs/msg/joy.hpp"

namespace obelisk {
    using unitree_fsm_msg             = obelisk_control_msgs::msg::ExecutionFSM;
    using unitree_high_level_ctrl_msg = obelisk_control_msgs::msg::VelocityCommand;

    class UnitreeJoystick : public ObeliskController<unitree_high_level_ctrl_msg, sensor_msgs::msg::Joy> {
      public:
        UnitreeJoystick(const std::string& name)
            : ObeliskController<unitree_high_level_ctrl_msg, sensor_msgs::msg::Joy>(name) {
            // Set velocity bounds
            this->declare_parameter<float>("v_x_max", 0.5);
            this->declare_parameter<float>("v_y_max", 0.5);
            this->declare_parameter<float>("w_z_max", 0.5);
            v_x_max_ = this->get_parameter("v_x_max").as_double();
            v_y_max_ = this->get_parameter("v_y_max").as_double();
            w_z_max_ = this->get_parameter("w_z_max").as_double();

            // Handle Execution FSM buttons
            // Buttons with negative values will corrospond to the DPAD
            this->declare_parameter<int>("unitree_home_button", -1);
            this->declare_parameter<int>("user_pose_button", 6);
            this->declare_parameter<int>("low_level_ctrl_button", -1);
            this->declare_parameter<int>("high_level_ctrl_button", -1);
            this->declare_parameter<int>("damping_button", -1);
            unitree_home_button_    = this->get_parameter("unitree_home_button").as_int();
            user_pose_button_       = this->get_parameter("user_pose_button").as_int();
            low_level_ctrl_button_  = this->get_parameter("low_level_ctrl_button").as_int();
            high_level_ctrl_button_ = this->get_parameter("high_level_ctrl_button").as_int();
            damping_button_         = this->get_parameter("damping_button").as_int();
            if (unitree_home_button_ > 10 || low_level_ctrl_button_ > 10 || high_level_ctrl_button_ > 10 ||
                damping_button_ > 10 || user_pose_button_ > 10 || user_pose_button_ < 0) {
                throw std::runtime_error("[UnitreeJoystick] Execution FSM buttons exceed number of buttons (0-10)!");
            }

            // Register publishers
            this->RegisterObkPublisher<unitree_fsm_msg>("pub_exec_fsm_setting", pub_exec_fsm_key_);
            RCLCPP_INFO_STREAM(this->get_logger(), "UnitreeJoystick node has been initialized.");
        }

      protected:
        void UpdateXHat(__attribute__((unused)) const sensor_msgs::msg::Joy& msg) override {
            // Update velocity commands
            constexpr int LEFT_STICK_X  = 0;
            constexpr int LEFT_STICK_Y  = 1;
            constexpr int RIGHT_STICK_X = 3;

            unitree_high_level_ctrl_msg high_level_msg;
            high_level_msg.header.stamp = this->now();
            high_level_msg.v_x          = msg.axes[LEFT_STICK_Y] * v_x_max_;
            high_level_msg.v_y          = msg.axes[LEFT_STICK_X] * v_y_max_;
            high_level_msg.w_z          = msg.axes[RIGHT_STICK_X] * w_z_max_;

            this->GetPublisher<unitree_high_level_ctrl_msg>(this->ctrl_key_)->publish(high_level_msg);

            // ----- Axes ----- //
            constexpr int DPAD_VERTICAL   = 7;
            constexpr int DPAD_HORIZONTAL = 6;
            constexpr int MENU            = 7;

            // Print control menu
            static rclcpp::Time last_menu_press = this->now();
            if ((this->now() - last_menu_press).seconds() > 1 && msg.buttons[MENU]) {
                RCLCPP_INFO_STREAM(
                    this->get_logger(),
                    "UnitreeJoystick Button Layout (XBox Controller):\n"
                        << "Execution Finite State Machine:\n"
                        << "\tHOME:            "
                        << ((unitree_home_button_ < 0) ? "DPAD_LEFT" : BUTTON_NAMES[unitree_home_button_]) << "\n"
                        << "\tDAMPING:         "
                        << ((damping_button_ < 0) ? "DPAD_RIGHT" : BUTTON_NAMES[damping_button_]) << "\n"
                        << "\tLOW_LEVEL_CTRL:  "
                        << ((low_level_ctrl_button_ < 0) ? "DPAD_DOWN" : BUTTON_NAMES[low_level_ctrl_button_]) << "\n"
                        << "\tHIGH_LEVEL_CTRL: "
                        << ((high_level_ctrl_button_ < 0) ? "DPAD_UP" : BUTTON_NAMES[high_level_ctrl_button_]) << "\n"
                        << "High Level Velocity Commands:\n"
                        << "\tFront/Back: Left Stick Vertical\n"
                        << "\tSide/Side:  Left Stick Horizontal\n"
                        << "\tYaw Rate:   Right Stick Horizontal");
                last_menu_press = this->now();
            }

            // Trigger FSM
            static rclcpp::Time last_Dpad_press = this->now();
            if ((this->now() - last_Dpad_press).seconds() > 1) {
                unitree_fsm_msg fsm_msg;
                fsm_msg.header.stamp = this->now();

                if ((msg.axes[DPAD_HORIZONTAL] < -0.5 && damping_button_ < 0) ||
                    (damping_button_ >= 0 && msg.buttons[damping_button_])) {
                    fsm_msg.cmd_exec_fsm_state = 5; // Damping
                } else if ((msg.axes[DPAD_HORIZONTAL] > 0.5 && unitree_home_button_ < 0) ||
                           (unitree_home_button_ >= 0 && msg.buttons[unitree_home_button_])) {
                    fsm_msg.cmd_exec_fsm_state = 1; // Home
                } else if ((msg.axes[DPAD_VERTICAL] > 0.5 && high_level_ctrl_button_ < 0) ||
                           (high_level_ctrl_button_ >= 0 && msg.buttons[high_level_ctrl_button_])) {
                    fsm_msg.cmd_exec_fsm_state = 4; // High Level Contrl
                } else if ((msg.axes[DPAD_VERTICAL] < -0.5 && low_level_ctrl_button_ < 0) ||
                           (low_level_ctrl_button_ >= 0 && msg.buttons[low_level_ctrl_button_])) {
                    fsm_msg.cmd_exec_fsm_state = 3; // Low Level Control
                } else if (msg.buttons[user_pose_button_]) {
                    fsm_msg.cmd_exec_fsm_state = 2; // User Pose
                } else {
                    return;
                }
                last_Dpad_press = this->now();
                this->GetPublisher<unitree_fsm_msg>(pub_exec_fsm_key_)->publish(fsm_msg);
                RCLCPP_INFO_STREAM(this->get_logger(), "UnitreeJoystick sent Execution FSM Command.");
            }
        }

        unitree_high_level_ctrl_msg ComputeControl() override {
            // This callback is not used. We send the high level control in the callback,
            // for safety if the remote is disconnected
            unitree_high_level_ctrl_msg msg;
            return msg;
        };

        // Publisher key
        const std::string pub_exec_fsm_key_ = "pub_exec_fsm_key";

        // Hold velocity bounds
        float v_x_max_;
        float v_y_max_;
        float w_z_max_;

        // Hold button locations for execution fsm
        int damping_button_;
        int unitree_home_button_;
        int user_pose_button_;
        int low_level_ctrl_button_;
        int high_level_ctrl_button_;

      private:
        // Button names
        const std::array<std::string, 11> BUTTON_NAMES = {"A",    "B",    "X",      "Y",      "LB",   "RB",
                                                          "VIEW", "MENU", "LSTICK", "RSTICK", "SHARE"};
    };
} // namespace obelisk
