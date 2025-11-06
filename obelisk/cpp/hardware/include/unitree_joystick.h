#include "obelisk_controller.h"
#include "obelisk_ros_utils.h"
#include "sensor_msgs/msg/joy.hpp"
#include "obelisk_control_msgs/msg/execution_fsm.hpp"
#include "obelisk_control_msgs/msg/velocity_command.hpp"


namespace obelisk {
    using unitree_fsm_msg = obelisk_control_msgs::msg::ExecutionFSM;
    using vel_cmd_msg = obelisk_control_msgs::msg::VelocityCommand;

    class UnitreeJoystick : public ObeliskController<vel_cmd_msg, sensor_msgs::msg::Joy> {
      public:
        UnitreeJoystick(const std::string& name)
            : ObeliskController<vel_cmd_msg, sensor_msgs::msg::Joy>(name) {
            // Set velocity bounds
            this->declare_parameter<float>("v_x_scale", 0.5);
            this->declare_parameter<float>("v_y_scale", 0.5);
            this->declare_parameter<float>("w_z_scale", 0.5);
            v_x_scale_ = this->get_parameter("v_x_scale").as_double();
            v_y_scale_ = this->get_parameter("v_y_scale").as_double();
            w_z_scale_ = this->get_parameter("w_z_scale").as_double();

            // Handle Execution FSM buttons
            // Buttons with negative values will corrospond to the DPAD
            this->declare_parameter<int>("unitree_home_button", ButtonMap::DR1);
            unitree_home_button_ = this->get_parameter("unitree_home_button").as_int();
            unitree_home_button_on_layer_ = isOnLayer(unitree_home_button_);
            unitree_home_button_on_axis_ = isOnAxis(unitree_home_button_);
            unitree_home_button_on_dpad_ = isOnDpad(unitree_home_button);
            if !recognizedButton(unitree_home_button) throw std::runtime_error("[UnitreeJoystick] Unitree Home Button " << unitree_home_button_ << " not recognized!!");
            
            this->declare_parameter<int>("user_pose_button", ButtonMap::DL1);
            user_pose_button_ = this->get_parameter("user_pose_button").as_int();
            user_pose_button_on_layer_ = isOnLayer(user_pose_button_);
            user_pose_button_on_axis_ = isOnAxis(user_pose_button_);
            user_pose_button_on_dpad_ = isOnDpad(user_pose_button_);
            if !recognizedButton(user_pose_button_) throw std::runtime_error("[UnitreeJoystick] Unitree User Pose Button " << user_pose_button_ << " not recognized!!");
            
            this->declare_parameter<int>("low_level_ctrl_button", ButtonMap::DD1);
            low_level_ctrl_button_ = this->get_parameter("low_level_ctrl_button").as_int();
            low_level_ctrl_button_on_layer_ = isOnLayer(low_level_ctrl_button_);
            low_level_ctrl_button_on_axis_ = isOnAxis(low_level_ctrl_button_);
            low_level_ctrl_button_on_dpad_ = isOnDpad(low_level_ctrl_button_);
            if !recognizedButton(low_level_ctrl_button_) throw std::runtime_error("[UnitreeJoystick] Unitree Low Level Control Button " << low_level_ctrl_button_ << " not recognized!!");
            
            this->declare_parameter<int>("high_level_ctrl_button", ButtonMap::DU1);
            high_level_ctrl_button_ = this->get_parameter("high_level_ctrl_button").as_int();
            high_level_ctrl_button_on_layer_ = isOnLayer(high_level_ctrl_button_);
            high_level_ctrl_button_on_axis_ = isOnAxis(high_level_ctrl_button_);
            high_level_ctrl_button_on_dpad_ = isOnDpad(high_level_ctrl_button_);
            if !recognizedButton(high_level_ctrl_button_) throw std::runtime_error("[UnitreeJoystick] Unitree High Level Control Button " << high_level_ctrl_button_ << " not recognized!!");
            
            this->declare_parameter<int>("damping_button", ButtonMap::LT);
            damping_button_ = this->get_parameter("damping_button").as_int();
            damping_button_on_layer_ = isOnLayer(damping_button_);
            damping_button_on_axis_ = isOnAxis(damping_button_);
            damping_button_on_dpad_ = isOnDpad(damping_button_);
            if !recognizedButton(damping_button_) throw std::runtime_error("[UnitreeJoystick] Unitree Damping Button " << damping_button_ << " not recognized!!");
            
            this->declare_parameter<int>("estop", ButtonMap::RT);
            estop_ = this->get_parameter("estop").as_int();
            estop_on_layer_ = isOnLayer(estop_);
            estop_on_axis_ = isOnAxis(estop_);
            estop_on_dpad_ = isOnDpad(estop_);
            if !recognizedButton(estop_) throw std::runtime_error("[UnitreeJoystick] Unitree ESTOP Button " << estop_ << " not recognized!!");
            
            this->declare_parameter<int>("vel_cmd_x", ButtonMap::LY);
            vel_cmd_x_ = this->get_parameter("vel_cmd_x").as_int();
            vel_cmd_x_on_layer_ = isOnLayer(vel_cmd_x_);
            vel_cmd_x_on_axis_ = isOnAxis(vel_cmd_x_);
            vel_cmd_x_on_dpad_ = isOnDpad(vel_cmd_x_);
            if !recognizedButton(vel_cmd_x_) throw std::runtime_error("[UnitreeJoystick] Unitree x vel axis " << vel_cmd_x_ << " not recognized!!");
            if !vel_cmd_x_on_axis_ throw std::runtime_error("[UnitreeJoystick] Unitree x vel axis " << vel_cmd_x_ << " is not on a joystick axis!!");
            
            this->declare_parameter<int>("vel_cmd_y", ButtonMap::LX);
            vel_cmd_y_ = this->get_parameter("vel_cmd_y").as_int();
            vel_cmd_y_on_layer_ = isOnLayer(vel_cmd_y_);
            vel_cmd_y_on_axis_ = isOnAxis(vel_cmd_y_);
            vel_cmd_y_on_dpad_ = isOnDpad(vel_cmd_y_);
            if !recognizedButton(vel_cmd_y_) throw std::runtime_error("[UnitreeJoystick] Unitree y vel axis " << vel_cmd_y_ << " not recognized!!");
            if !vel_cmd_y_on_axis_ throw std::runtime_error("[UnitreeJoystick] Unitree y vel axis " << vel_cmd_y_ << " is not on a joystick axis!!");
            
            this->declare_parameter<int>("vel_cmd_yaw", ButtonMap::RY);
            vel_cmd_yaw_ = this->get_parameter("vel_cmd_yaw").as_int();
            vel_cmd_yaw_on_layer_ = isOnLayer(vel_cmd_yaw_);
            vel_cmd_yaw_on_axis_ = isOnAxis(vel_cmd_yaw_);
            vel_cmd_yaw_on_dpad_ = isOnDpad(vel_cmd_yaw_);
            if !recognizedButton(vel_cmd_yaw_) throw std::runtime_error("[UnitreeJoystick] Unitree yaw vel axis " << vel_cmd_yaw_ << " not recognized!!");
            if !vel_cmd_yaw_on_axis_ throw std::runtime_error("[UnitreeJoystick] Unitree yaw vel axis " << vel_cmd_yaw_ << " is not on a joystick axis!!");
            
            this->declare_parameter<int>("layer_button", ButtonMap::LB);
            layer_button = this->get_parameter("layer_button");
            layer_buttonon_axis_ = isOnAxis(layer_button);
            layer_buttonon_dpad_ = isOnDpad(layer_button);
            if !recognizedButton(layer_button) throw std::runtime_error("[UnitreeJoystick] Unitree Layer Button " << layer_button << " not recognized!!");
            
            std::initializer_list<int> values = {estop_, damping_button_, user_pose_button_, unitree_home_button_, low_level_ctrl_button_, high_level_ctrl_button_, vel_cmd_x_, vel_cmd_y_, vel_cmd_yaw_, layer_button};
            std::unordered_set<int> s(values.begin(), values.end());
            if !(s.size() == values.size()) throw std::runtime_error("[UnitreeJoystick] Unitree Joystick Configuration contains duplicate buttons " << values);


            // Register publishers
            this->RegisterObkPublisher<unitree_fsm_msg>("pub_exec_fsm_setting", pub_exec_fsm_key_);
            RCLCPP_INFO_STREAM(this->get_logger(), "UnitreeJoystick node has been initialized.");
        }
        static const int LAYER_OFFSET = 100;
        static const int AXIS_OFFSET = 10;
        static const int NEG_OFFSET = 11;
        static const enum class ButtonMap : uint8_t {
            LT = 2 + AXIS_OFFSET,
            RT = 5 + AXIS_OFFSET,
            LB = 4,
            RB = 5,
            X = 3,
            Y = 2,
            A = 0,
            B = 1,
            DL = 6 + AXIS_OFFSET + NEG_OFFSET,
            DR = 6 + AXIS_OFFSET,
            DU = 7 + AXIS_OFFSET,
            DD = 7 + AXIS_OFFSET + NEG_OFFSET,
            LX = 0 + AXIS_OFFSET,
            LY = 1 + AXIS_OFFSET,
            RX = 3 + AXIS_OFFSET,
            RY = 4 + AXIS_OFFSET,
            MENU = 7,

            // Modified by layer button
            X1 = X + LAYER_OFFSET,
            Y1 = Y + LAYER_OFFSET,
            A1 = A + LAYER_OFFSET,
            B1 = B + LAYER_OFFSET,
            DL1 = DL + LAYER_OFFSET,
            DR1 = DR + LAYER_OFFSET,
            DU1 = DU + LAYER_OFFSET,
            DD1 = DD + LAYER_OFFSET,
        };

        static const std::unordered_map<ButtonMap, std::string> ButtonNames = {
            {ButtonMap::LT, "Left Trigger"},
            {ButtonMap::RT, "Right Trigger"},
            {ButtonMap::LB, "Left Button"},
            {ButtonMap::RB, "Right Button"},
            {ButtonMap::X, "X"},
            {ButtonMap::Y, "Y"},
            {ButtonMap::A, "A"},
            {ButtonMap::B, "B"},
            {ButtonMap::DL, "DPad Left"},
            {ButtonMap::DR, "DPad Right"},
            {ButtonMap::DU, "DPad Up"},
            {ButtonMap::DD, "DPad Down"},
            {ButtonMap::LX, "Left Stick X"},
            {ButtonMap::LY, "Left Stick Y"},
            {ButtonMap::RX, "Right Stick X"},
            {ButtonMap::RY, "Right Stick Y"},
            {ButtonMap::MENU, "Menu"},
            {ButtonMap::X1, "Layer 1 X"},
            {ButtonMap::Y1, "Layer 1 Y"},
            {ButtonMap::A1, "Layer 1 A"},
            {ButtonMap::B1, "Layer 1 B"},
            {ButtonMap::DL1, "Layer 1 DPad Left"},
            {ButtonMap::DR1, "Layer 1 DPad Right"},
            {ButtonMap::DU1, "Layer 1 DPad Up"},
            {ButtonMap::DD1, "Layer 1 DPad Down"},
        };

      protected:
        void UpdateXHat(__attribute__((unused)) const sensor_msgs::msg::Joy& msg) override {
            vel_cmd_msg vel_msg;
            vel_msg.header.stamp = this->now();
            vel_msg.v_x = msg.axes[ButtonMap::LY] * v_x_scale_;
            vel_msg.v_y = msg.axes[ButtonMap::LX] * v_y_scale_;
            vel_msg.w_z = msg.axes[ButtonMap::RX] * w_z_scale_;

            this->GetPublisher<vel_cmd_msg>(this->ctrl_key_)->publish(vel_msg);

            // ----- Axes ----- //
            constexpr int DPAD_VERTICAL = 7;
            constexpr int DPAD_HORIZONTAL = 6;
            
            // Print control menu
            static rclcpp::Time last_menu_press = this->now();
            if ((this->now() - last_menu_press).seconds() > 1 && msg.buttons[ButtonMap::MENU]) {
                RCLCPP_INFO_STREAM(this->get_logger(),
                    "UnitreeJoystick Button Layout (XBox Controller):\n" <<
                    "Execution Finite State Machine:\n" << 
                    "\tHOME:            " << ButtonNames[unitree_home_button_] << "\n" << 
                    "\tDAMPING:         " << ButtonNames[damping_button_] << "\n" <<
                    "\tLOW_LEVEL_CTRL:  " << ButtonNames[low_level_ctrl_button_] << "\n" <<
                    "\tHIGH_LEVEL_CTRL: " << ButtonNames[high_level_ctrl_button_] << "\n" <<
                    "High Level Velocity Commands:\n" <<
                    "\tFront/Back: " << ButtonNames[vel_cmd_x_] << "\n" <<
                    "\tSide/Side:  " << ButtonNames[vel_cmd_y_] << "\n" <<
                    "\tYaw Rate:   " << ButtonNames[vel_cmd_yaw_] << "";
                last_menu_press = this->now();
            }

            // Trigger FSM
            static rclcpp::Time last_fsm_msg = this->now();
            if ((this->now() - last_fsm_msg).seconds() > 1) {
                unitree_fsm_msg fsm_msg;
                fsm_msg.header.stamp = this->now();
                
                if getEstop(msg) {
                    fsm_msg.cmd_exec_fsm_state = 5;    // ESTOP
                } else if getDamping(msg) {
                    fsm_msg.cmd_exec_fsm_state = 5;    // Damping
                } else if getUserPose(msg) {
                    fsm_msg.cmd_exec_fsm_state = 5;    // User Pose
                } else if getUnitreeHome(msg) {
                    fsm_msg.cmd_exec_fsm_state = 5;    // Unitree Stand
                } else if getUserCtrl(msg) {
                    fsm_msg.cmd_exec_fsm_state = 5;    // User Control
                } else if getUnitreeCtrl(msg) {
                    fsm_msg.cmd_exec_fsm_state = 5;    // Unitree Control
                } else {
                    continue;
                }
                last_Dpad_press = this->now();
                this->GetPublisher<unitree_fsm_msg>(pub_exec_fsm_key_)->publish(fsm_msg);
                RCLCPP_INFO_STREAM(this->get_logger(), "UnitreeJoystick sent Execution FSM Command.");
            }
        }

        bool recognizeButton(ButtonMap btn) {
            return ButtonNames.count(btn) > 0
        }

        bool getEstopButton(const sensor_msgs::msg::Joy& msg) {
            return getButton(msg, estop_, estop_on_layer_, estop_on_axis_, estop_on_dpad_);
        }

        bool getDampingButton(const sensor_msgs::msg::Joy& msg) {
            return getButton(msg, damping_button_, damping_button_on_layer_, damping_button_on_axis_, damping_button_on_dpad_);
        }

        bool getUserPoseButton(const sensor_msgs::msg::Joy& msg) {
            return getButton(msg, user_pose_button_, user_pose_button_on_layer_, user_pose_button_on_axis_, user_pose_button_on_dpad_);
        }

        bool getUnitreeHomeButton(const sensor_msgs::msg::Joy& msg) {
            return getButton(msg, unitree_home_button_, unitree_home_button_on_layer_, unitree_home_button_on_axis_, unitree_home_button_on_dpad_);
        }

        bool getUserCtrlButton(const sensor_msgs::msg::Joy& msg) {
            return getButton(msg, low_level_ctrl_button_, low_level_ctrl_button_on_layer_, low_level_ctrl_button_on_axis_, low_level_ctrl_on_dpad_);
        }
        bool getUnitreeCtrlButton(const sensor_msgs::msg::Joy& msg) {
            return getButton(msg, high_level_ctrl_button_, high_level_ctrl_button_on_layer_, high_level_ctrl_button_on_axis_, high_level_ctrl_button_on_dpad_);
        }

        bool getButton(const sensor_msgs::msg::Joy& msg, ButtonMap btn, bool on_layer, bool on_axis, bool on_dpad){
            if on_layer {
                if !get_button(msg, layer_button, false, layer_on_axis, layer_on_dpad) {
                    return false;
                }
                btn -= LAYER_OFFSET;
            }
            if on_axis {
                return msg.axis[btn - AXIS_OFFSET] > axis_threshold_;
            }
            if on_dpad {
                int sgn = 1;
                if (btn == ButtonMap::DL || btn == ButtonMap::DD || btn == ButtonMap::DL1 || btn == ButtonMap::DD1) {
                    btn -= NEG_OFFSET;
                    sgn = -1;
                }
                return msg.axis[btn - AXIS_OFFSET] * sgn > 0.5
            }
            return msg.buttons[btn];
        }

        bool isOnLayer(ButtonMap btn) {
            return btn == ButtonMap::X1 || btn == ButtonMap::Y1 || btn == ButtonMap::A1 || btn == ButtonMap::B1 || btn == ButtonMap::DL1 || btn == ButtonMap::DR1 || btn == ButtonMap::DU1 || btn == ButtonMap::DD1;
        }

        bool isOnAxis(ButtonMap btn) {
            return btn == ButtonMap::LT || btn == ButtonMap::RT || btn == ButtonMap::LX || btn == ButtonMap::LY || btn == ButtonMap::RX || btn == ButtonMap::RY;
        }

        bool isOnDpad(ButtonMap btn) {
            return btn == ButtonMap::DL || btn == ButtonMap::DR || btn == ButtonMap::DU || btn == ButtonMap::DD || btn == ButtonMap::DL1 || btn == ButtonMap::DR1 || btn == ButtonMap::DU1 || btn == ButtonMap::DD1;
        }

        vel_cmd_msg ComputeControl() override {
            // This callback is not used. We send the high level control in the callback,
            // for safety if the remote is disconnected
            vel_cmd_msg msg;
            return msg;
        };

        // Publisher key
        const std::string pub_exec_fsm_key_ = "pub_exec_fsm_key";

        // Hold velocity bounds
        float v_x_scale_;
        float v_y_scale_; 
        float w_z_scale_;

        // Hold button locations for execution fsm
        int damping_button_;
        int unitree_home_button_;
        int user_pose_button_;
        int low_level_ctrl_button_;
        int high_level_ctrl_button_;
        int estop_;
        int layer_button;
        bool damping_button_on_layer_;
        bool unitree_home_button_on_layer_;
        bool user_pose_button_on_layer_;
        bool low_level_ctrl_button_on_layer_;
        bool high_level_ctrl_button_on_layer_;
        bool estop_on_layer_;
        bool damping_button_on_axis_;
        bool unitree_home_button_on_axis_;
        bool user_pose_button_on_axis_;
        bool low_level_ctrl_button_on_axis_;
        bool high_level_ctrl_button_on_axis_;
        bool estop_on_axis_;
        bool damping_button_on_dpad_;
        bool unitree_home_button_on_dpad_;
        bool user_pose_button_on_dpad_;
        bool low_level_ctrl_button_on_dpad_;
        bool high_level_ctrl_button_on_dpad_;
        bool layer_button_on_dpad_;
        bool layer_button_on_axis;
        bool estop_on_dpad_;

        int vel_cmd_x_;
        int vel_cmd_y_;
        int vel_cmd_yaw_;
        bool vel_cmd_x_on_layer_;
        bool vel_cmd_y_on_layer_;
        bool vel_cmd_yaw_on_layer_;
        bool vel_cmd_x_on_axis_;
        bool vel_cmd_y_on_axis_;
        bool vel_cmd_yaw_on_axis_;
        bool vel_cmd_x_on_dpad_;
        bool vel_cmd_y_on_dpad_;
        bool vel_cmd_yaw_on_dpad_;
      private:
    };
}