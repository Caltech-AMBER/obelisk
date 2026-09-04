#include <string>
#include <stdexcept>
#include <unordered_set>
#include <unordered_map>
#include <initializer_list>

#include "obelisk_controller.h"
#include "obelisk_ros_utils.h"
#include "sensor_msgs/msg/joy.hpp"
#include "obelisk_control_msgs/msg/execution_fsm.hpp"
#include "obelisk_control_msgs/msg/velocity_command.hpp"
#include "unitree_fsm.h"
#include "joy_debounce.h"


namespace obelisk {
    using unitree_fsm_msg = obelisk_control_msgs::msg::ExecutionFSM;
    using vel_cmd_msg = obelisk_control_msgs::msg::VelocityCommand;

    class UnitreeJoystick : public ObeliskController<vel_cmd_msg, sensor_msgs::msg::Joy> {
      public:
        static const int LAYER_OFFSET = 100;
        static const int AXIS_OFFSET = 10;
        static const int NEG_OFFSET = 11;
        enum class ButtonMap : uint8_t {
            LT = 2 + AXIS_OFFSET,
            RT = 5 + AXIS_OFFSET,
            LB = 4,
            RB = 5,
            X = 3,
            Y = 2,
            A = 0,
            B = 1,
            DR = 6 + AXIS_OFFSET + NEG_OFFSET,
            DL = 6 + AXIS_OFFSET,
            DU = 7 + AXIS_OFFSET,
            DD = 7 + AXIS_OFFSET + NEG_OFFSET,
            LX = 0 + AXIS_OFFSET,
            LY = 1 + AXIS_OFFSET,
            RX = 3 + AXIS_OFFSET,
            RY = 4 + AXIS_OFFSET,
            MENU = 7,

            // Modified by layer button
            X1 = static_cast<int>(X) + LAYER_OFFSET,
            Y1 = static_cast<int>(Y) + LAYER_OFFSET,
            A1 = static_cast<int>(A) + LAYER_OFFSET,
            B1 = static_cast<int>(B) + LAYER_OFFSET,
            DR1 = static_cast<int>(DR) + LAYER_OFFSET,
            DL1 = static_cast<int>(DL) + LAYER_OFFSET,
            DU1 = static_cast<int>(DU) + LAYER_OFFSET,
            DD1 = static_cast<int>(DD) + LAYER_OFFSET,
        };

        UnitreeJoystick(const std::string& name)
            : ObeliskController<vel_cmd_msg, sensor_msgs::msg::Joy>(name) {
            // Set velocity bounds
            this->declare_parameter<float>("v_x_scale", 0.5);
            this->declare_parameter<float>("v_y_scale", 0.5);
            this->declare_parameter<float>("w_z_scale", 0.5);
            v_x_scale_ = this->get_parameter("v_x_scale").as_double();
            v_y_scale_ = this->get_parameter("v_y_scale").as_double();
            w_z_scale_ = this->get_parameter("w_z_scale").as_double();
            this->declare_parameter<float>("axis_threshold", -0.1);
            axis_threshold_ = this->get_parameter("axis_threshold").as_double();

            // Spurious-press filter for FSM bindings that live on an ANALOG axis (the
            // triggers). See joy_debounce.h for the measurements behind the defaults and
            // for why digital bindings are deliberately left alone. Restore the old
            // unfiltered behaviour with axis_press_threshold = axis_threshold and
            // axis_confirm_samples = 1.
            this->declare_parameter<float>("axis_press_threshold", -0.5);
            axis_press_threshold_ = this->get_parameter("axis_press_threshold").as_double();
            this->declare_parameter<float>("axis_release_threshold", -0.15);
            axis_release_threshold_ = this->get_parameter("axis_release_threshold").as_double();
            this->declare_parameter<int>("axis_confirm_samples", 2);
            axis_confirm_samples_ = this->get_parameter("axis_confirm_samples").as_int();
            // The Schmitt band is [press, release] with press the DEEPER (more negative)
            // end. Inverting them would make the filter latch on and never release, so
            // fail at construction rather than at the first pull.
            if (axis_release_threshold_ < axis_press_threshold_) throw std::runtime_error(
                std::string("[UnitreeJoystick] axis_release_threshold (")
                + std::to_string(axis_release_threshold_)
                + std::string(") must be shallower than axis_press_threshold (")
                + std::to_string(axis_press_threshold_) + std::string(")")
            );
            if (axis_confirm_samples_ < 1) throw std::runtime_error(
                std::string("[UnitreeJoystick] axis_confirm_samples must be >= 1, got ")
                + std::to_string(axis_confirm_samples_)
            );

            // Handle Execution FSM buttons
            this->declare_parameter<int>("menu_button", static_cast<int>(ButtonMap::MENU));
            menu_button_ = this->get_parameter("menu_button").as_int();
            menu_button_on_layer_ = isOnLayer(menu_button_);
            menu_button_on_axis_ = isOnAxis(menu_button_);
            menu_button_on_dpad_ = isOnDpad(menu_button_);
            if (!recognizeButton(menu_button_)) throw std::runtime_error(
                std::string("[UnitreeJoystick] Menu Button ") + std::to_string(menu_button_) + std::string(" not recognized!!")
            );

            this->declare_parameter<int>("unitree_home_button", static_cast<int>(ButtonMap::DR1));
            unitree_home_button_ = this->get_parameter("unitree_home_button").as_int();
            unitree_home_button_on_layer_ = isOnLayer(unitree_home_button_);
            unitree_home_button_on_axis_ = isOnAxis(unitree_home_button_);
            unitree_home_button_on_dpad_ = isOnDpad(unitree_home_button_);
            if (!recognizeButton(unitree_home_button_)) throw std::runtime_error(
                std::string("[UnitreeJoystick] Unitree Home Button ") + std::to_string(unitree_home_button_) + std::string(" not recognized!!")
            );
            
            this->declare_parameter<int>("user_pose_button", static_cast<int>(ButtonMap::DL1));
            user_pose_button_ = this->get_parameter("user_pose_button").as_int();
            user_pose_button_on_layer_ = isOnLayer(user_pose_button_);
            user_pose_button_on_axis_ = isOnAxis(user_pose_button_);
            user_pose_button_on_dpad_ = isOnDpad(user_pose_button_);
            if (!recognizeButton(user_pose_button_)) throw std::runtime_error(
                std::string("[UnitreeJoystick] Unitree User Pose Button ") + std::to_string(user_pose_button_) + std::string(" not recognized!!")
            );
            
            this->declare_parameter<int>("low_level_ctrl_button", static_cast<int>(ButtonMap::DD1));
            low_level_ctrl_button_ = this->get_parameter("low_level_ctrl_button").as_int();
            low_level_ctrl_button_on_layer_ = isOnLayer(low_level_ctrl_button_);
            low_level_ctrl_button_on_axis_ = isOnAxis(low_level_ctrl_button_);
            low_level_ctrl_button_on_dpad_ = isOnDpad(low_level_ctrl_button_);
            if (!recognizeButton(low_level_ctrl_button_)) throw std::runtime_error(
                std::string("[UnitreeJoystick] Unitree Low Level Control Button ") + std::to_string(low_level_ctrl_button_) + std::string(" not recognized!!")
            );
            
            this->declare_parameter<int>("high_level_ctrl_button", static_cast<int>(ButtonMap::DU1));
            high_level_ctrl_button_ = this->get_parameter("high_level_ctrl_button").as_int();
            high_level_ctrl_button_on_layer_ = isOnLayer(high_level_ctrl_button_);
            high_level_ctrl_button_on_axis_ = isOnAxis(high_level_ctrl_button_);
            high_level_ctrl_button_on_dpad_ = isOnDpad(high_level_ctrl_button_);
            if (!recognizeButton(high_level_ctrl_button_)) throw std::runtime_error(
                std::string("[UnitreeJoystick] Unitree High Level Control Button ") + std::to_string(high_level_ctrl_button_) + std::string(" not recognized!!")
            );
            
            this->declare_parameter<int>("damping_button", static_cast<int>(ButtonMap::LT));
            damping_button_ = this->get_parameter("damping_button").as_int();
            damping_button_on_layer_ = isOnLayer(damping_button_);
            damping_button_on_axis_ = isOnAxis(damping_button_);
            damping_button_on_dpad_ = isOnDpad(damping_button_);
            if (!recognizeButton(damping_button_)) throw std::runtime_error(
                std::string("[UnitreeJoystick] Unitree Damping Button ") + std::to_string(damping_button_) + std::string(" not recognized!!")
            );
            
            this->declare_parameter<int>("estop", static_cast<int>(ButtonMap::RT));
            estop_ = this->get_parameter("estop").as_int();
            estop_on_layer_ = isOnLayer(estop_);
            estop_on_axis_ = isOnAxis(estop_);
            estop_on_dpad_ = isOnDpad(estop_);
            if (!recognizeButton(estop_)) throw std::runtime_error(
                std::string("[UnitreeJoystick] Unitree ESTOP Button ") + std::to_string(estop_) + std::string(" not recognized!!")
            );
            
            this->declare_parameter<int>("vel_cmd_x", static_cast<int>(ButtonMap::LY));
            vel_cmd_x_ = this->get_parameter("vel_cmd_x").as_int();
            vel_cmd_x_on_layer_ = isOnLayer(vel_cmd_x_);
            vel_cmd_x_on_axis_ = isOnAxis(vel_cmd_x_);
            vel_cmd_x_on_dpad_ = isOnDpad(vel_cmd_x_);
            if (!recognizeButton(vel_cmd_x_)) throw std::runtime_error(
                std::string("[UnitreeJoystick] Unitree x vel axis ") + std::to_string(vel_cmd_x_) + std::string(" not recognized!!")
            );
            if (!vel_cmd_x_on_axis_) throw std::runtime_error(
                std::string("[UnitreeJoystick] Unitree x vel axis ") + std::to_string(vel_cmd_x_) + std::string(" is not on a joystick axis!!")
            );
            
            this->declare_parameter<int>("vel_cmd_y", static_cast<int>(ButtonMap::LX));
            vel_cmd_y_ = this->get_parameter("vel_cmd_y").as_int();
            vel_cmd_y_on_layer_ = isOnLayer(vel_cmd_y_);
            vel_cmd_y_on_axis_ = isOnAxis(vel_cmd_y_);
            vel_cmd_y_on_dpad_ = isOnDpad(vel_cmd_y_);
            if (!recognizeButton(vel_cmd_y_)) throw std::runtime_error(
                std::string("[UnitreeJoystick] Unitree y vel axis ") + std::to_string(vel_cmd_y_) + std::string(" not recognized!!")
            );
            if (!vel_cmd_y_on_axis_) throw std::runtime_error(
                std::string("[UnitreeJoystick] Unitree y vel axis ") + std::to_string(vel_cmd_y_) + std::string(" is not on a joystick axis!!")
            );
            
            this->declare_parameter<int>("vel_cmd_yaw", static_cast<int>(ButtonMap::RY));
            vel_cmd_yaw_ = this->get_parameter("vel_cmd_yaw").as_int();
            vel_cmd_yaw_on_layer_ = isOnLayer(vel_cmd_yaw_);
            vel_cmd_yaw_on_axis_ = isOnAxis(vel_cmd_yaw_);
            vel_cmd_yaw_on_dpad_ = isOnDpad(vel_cmd_yaw_);
            if (!recognizeButton(vel_cmd_yaw_)) throw std::runtime_error(
                std::string("[UnitreeJoystick] Unitree yaw vel axis ") + std::to_string(vel_cmd_yaw_) + std::string(" not recognized!!")
            );
            if (!vel_cmd_yaw_on_axis_) throw std::runtime_error(
                std::string("[UnitreeJoystick] Unitree yaw vel axis ") + std::to_string(vel_cmd_yaw_) + std::string(" is not on a joystick axis!!")
            );
            
            this->declare_parameter<int>("layer_button", static_cast<int>(ButtonMap::LB));
            layer_button_ = this->get_parameter("layer_button").as_int();
            layer_button_on_axis_ = isOnAxis(layer_button_);
            layer_button_on_dpad_ = isOnDpad(layer_button_);
            if (!recognizeButton(layer_button_)) throw std::runtime_error(
                std::string("[UnitreeJoystick] Unitree Layer Button ") + std::to_string(layer_button_) + std::string(" not recognized!!")
            );
            
            std::initializer_list<int> values = {estop_, damping_button_, user_pose_button_, unitree_home_button_, low_level_ctrl_button_, high_level_ctrl_button_, vel_cmd_x_, vel_cmd_y_, vel_cmd_yaw_, layer_button_};
            std::unordered_set<int> s(values.begin(), values.end());
            if (!(s.size() == values.size())) throw std::runtime_error(
                "[UnitreeJoystick] Duplicate buttons: set size "
                + std::to_string(s.size()) + " vs values "
                + std::to_string(values.size())
            );


            // One filter per FSM binding, so a pad with two triggers bound cannot have one
            // binding's confirmation count disturbed by the other.
            for (AxisPressFilter* f : {&estop_filter_, &damping_filter_, &user_pose_filter_,
                                       &unitree_home_filter_, &low_level_ctrl_filter_,
                                       &high_level_ctrl_filter_}) {
                f->Configure(axis_press_threshold_, axis_release_threshold_,
                             axis_confirm_samples_, axis_threshold_);
            }

            // Register publishers
            this->RegisterObkPublisher<unitree_fsm_msg>(pub_exec_fsm_key_);
            this->RegisterObkPublisher<sensor_msgs::msg::Joy>(pub_joy_passthrough_key_);
            RCLCPP_INFO_STREAM(this->get_logger(), "UnitreeJoystick node has been initialized.");
        }
        
        inline static const std::unordered_map<ButtonMap, std::string> BUTTON_NAMES = {
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
            // Evaluate EVERY FSM binding exactly once per message, up front and
            // unconditionally, BEFORE any branching on the results.
            //
            // The bindings on an analog axis are stateful now (joy_debounce.h counts
            // consecutive samples past the press threshold), and the branches below are
            // an if/else chain plus a 0.5 s rate gate -- so evaluating a binding inside
            // its own branch would only feed the filter on the ticks where every earlier
            // binding happened to be idle and the gate happened to be open, leaving it
            // with a stale confirmation count. Reading them all here keeps every filter
            // fed on every sample and keeps the branching pure.
            const bool estop_pressed = getFilteredButton(
                msg, estop_filter_, estop_, estop_on_layer_, estop_on_axis_, estop_on_dpad_, "ESTOP");
            const bool damping_pressed = getFilteredButton(
                msg, damping_filter_, damping_button_, damping_button_on_layer_,
                damping_button_on_axis_, damping_button_on_dpad_, "DAMPING");
            const bool user_pose_pressed = getFilteredButton(
                msg, user_pose_filter_, user_pose_button_, user_pose_button_on_layer_,
                user_pose_button_on_axis_, user_pose_button_on_dpad_, "USER_POSE");
            const bool unitree_home_pressed = getFilteredButton(
                msg, unitree_home_filter_, unitree_home_button_, unitree_home_button_on_layer_,
                unitree_home_button_on_axis_, unitree_home_button_on_dpad_, "UNITREE_HOME");
            const bool user_ctrl_pressed = getFilteredButton(
                msg, low_level_ctrl_filter_, low_level_ctrl_button_, low_level_ctrl_button_on_layer_,
                low_level_ctrl_button_on_axis_, low_level_ctrl_button_on_dpad_, "USER_CTRL");
            const bool unitree_ctrl_pressed = getFilteredButton(
                msg, high_level_ctrl_filter_, high_level_ctrl_button_, high_level_ctrl_button_on_layer_,
                high_level_ctrl_button_on_axis_, high_level_ctrl_button_on_dpad_, "UNITREE_VEL_CTRL");

            // Trigger FSM
            // First, check for damping or estop
            bool commanded = false;
            unitree_fsm_msg fsm_msg;
            fsm_msg.header.stamp = this->now();
            if (estop_pressed) {
                fsm_msg.cmd_exec_fsm_state = static_cast<uint8_t>(ExecFSMState::ESTOP);             // ESTOP
                commanded = true;
            } else if (damping_pressed) {
                fsm_msg.cmd_exec_fsm_state = static_cast<uint8_t>(ExecFSMState::DAMPING);           // Damping
                commanded = true;
            }

            static rclcpp::Time last_fsm_msg = this->now();
            if ((this->now() - last_fsm_msg).seconds() > 0.5) {
                if (user_pose_pressed) {
                    fsm_msg.cmd_exec_fsm_state = static_cast<uint8_t>(ExecFSMState::USER_POSE);         // User Pose
                    commanded = true;
                } else if (unitree_home_pressed) {
                    fsm_msg.cmd_exec_fsm_state = static_cast<uint8_t>(ExecFSMState::UNITREE_HOME);      // Unitree Stand
                    commanded = true;
                } else if (user_ctrl_pressed) {
                    fsm_msg.cmd_exec_fsm_state = static_cast<uint8_t>(ExecFSMState::USER_CTRL);         // User Control
                    commanded = true;
                } else if (unitree_ctrl_pressed) {
                    fsm_msg.cmd_exec_fsm_state = static_cast<uint8_t>(ExecFSMState::UNITREE_VEL_CTRL);  // Unitree Control
                    commanded = true;
                }
                if (commanded) {
                    last_fsm_msg = this->now();
                }
            }
            if (commanded){
                this->GetPublisher<unitree_fsm_msg>(pub_exec_fsm_key_)->publish(fsm_msg);
                // Debounce the LOG ONLY (the publish above always fires -- damping/kill
                // stay un-gated). Keyed on the commanded state so a fresh command prints
                // immediately and only same-state repeats are suppressed.
                if (fsm_log_debounce_.ShouldLog(this->now(), fsm_msg.cmd_exec_fsm_state, FSM_LOG_DEBOUNCE_SEC)) {
                    RCLCPP_INFO_STREAM(this->get_logger(), "UnitreeJoystick sent Execution FSM Command " << TRANSITION_STRINGS.at(static_cast<ExecFSMState>(fsm_msg.cmd_exec_fsm_state)));
                }
            }

            // Handle printing menu
            static rclcpp::Time last_menu_press = this->now();
            if ((this->now() - last_menu_press).seconds() > 1 && msg.buttons[static_cast<size_t>(ButtonMap::MENU)]) {
                RCLCPP_INFO_STREAM(this->get_logger(),
                    "UnitreeJoystick Button Layout (XBox Controller):\n" <<
                    "Execution Finite State Machine:\n" << 
                    "\tHOME:            " << BUTTON_NAMES.at(static_cast<ButtonMap>(unitree_home_button_)) << "\n" << 
                    "\tDAMPING:         " << BUTTON_NAMES.at(static_cast<ButtonMap>(damping_button_)) << "\n" <<
                    "\tLOW_LEVEL_CTRL:  " << BUTTON_NAMES.at(static_cast<ButtonMap>(low_level_ctrl_button_)) << "\n" <<
                    "\tHIGH_LEVEL_CTRL: " << BUTTON_NAMES.at(static_cast<ButtonMap>(high_level_ctrl_button_)) << "\n" <<
                    "High Level Velocity Commands:\n" <<
                    "\tFront/Back: " << BUTTON_NAMES.at(static_cast<ButtonMap>(vel_cmd_x_)) << "\n" <<
                    "\tSide/Side:  " << BUTTON_NAMES.at(static_cast<ButtonMap>(vel_cmd_y_)) << "\n" <<
                    "\tYaw Rate:   " << BUTTON_NAMES.at(static_cast<ButtonMap>(vel_cmd_yaw_))
                );
                last_menu_press = this->now();
            }

            // Publish velocity
            vel_cmd_msg vel_msg;
            vel_msg.header.stamp = this->now();
            vel_msg.v_x = getAxisValue(msg, vel_cmd_x_ - AXIS_OFFSET) * v_x_scale_;
            vel_msg.v_y = getAxisValue(msg, vel_cmd_y_ - AXIS_OFFSET) * v_y_scale_;
            vel_msg.w_z = getAxisValue(msg, vel_cmd_yaw_ - AXIS_OFFSET) * w_z_scale_;

            this->GetPublisher<vel_cmd_msg>(this->ctrl_key_)->publish(vel_msg);

            // Pass through the joystick message
            sensor_msgs::msg::Joy passthrough_msg = msg;

            auto zero_axis_idx = [&](int ax_idx){
                if (ax_idx >= 0 && ax_idx < static_cast<int>(passthrough_msg.axes.size())) {
                    if ((ax_idx == static_cast<int>(ButtonMap::RT) - AXIS_OFFSET) | (ax_idx == static_cast<int>(ButtonMap::LT) - AXIS_OFFSET)) {
                        passthrough_msg.axes[ax_idx] = 1.0f;
                    } else {
                        passthrough_msg.axes[ax_idx] = 0.0f;
                    }
                }                    
            };
            auto zero_button_idx = [&](int bt_idx){
                if (bt_idx >= 0 && bt_idx < static_cast<int>(passthrough_msg.buttons.size()))
                    passthrough_msg.buttons[bt_idx] = 0;
            };

            auto clear_binding = [&](int code, bool on_layer, bool on_axis, bool on_dpad){
                // If binding is layered, zero the layer button as well (prevents combo)
                if (on_layer) {
                    if (!getButton(msg, layer_button_, false, layer_button_on_axis_, layer_button_on_dpad_)) {
                        return;
                    }
                    // clear the physical layer button
                    if (layer_button_on_axis_) {
                        zero_axis_idx(layer_button_ - AXIS_OFFSET);
                    } else if (layer_button_on_dpad_) {
                        zero_axis_idx(layer_button_ - AXIS_OFFSET); // dpad also lives in axes
                    } else {
                        zero_button_idx(layer_button_);
                    }
                    // reduce code to base (match how getButton() reduces)
                    code -= LAYER_OFFSET;
                }

                if (on_axis) {
                    // axis/trigger input lives in axes[]
                    zero_axis_idx(code - AXIS_OFFSET);
                    return;
                }

                if (on_dpad) {
                    // DPAD lives in axes; positive/negative handled via sign in detection
                    int ax = code - AXIS_OFFSET;
                    // For negative variants (Left/Down), normalize like getButton() does
                    if (code == static_cast<int>(ButtonMap::DR) ||
                        code == static_cast<int>(ButtonMap::DD) ||
                        code == static_cast<int>(ButtonMap::DR1) ||
                        code == static_cast<int>(ButtonMap::DD1)) {
                        ax -= NEG_OFFSET; // mirror your detection
                    }
                    zero_axis_idx(ax);
                    return;
                }

                // Plain digital button
                zero_button_idx(code);
            };

            // Clear ONLY the FSM-triggering controls (leave velocity axes intact)
            clear_binding(estop_,                 estop_on_layer_,                 estop_on_axis_,                 estop_on_dpad_);
            clear_binding(menu_button_,           menu_button_on_layer_,           menu_button_on_axis_,           menu_button_on_dpad_);
            clear_binding(damping_button_,        damping_button_on_layer_,        damping_button_on_axis_,        damping_button_on_dpad_);
            clear_binding(user_pose_button_,      user_pose_button_on_layer_,      user_pose_button_on_axis_,      user_pose_button_on_dpad_);
            clear_binding(unitree_home_button_,   unitree_home_button_on_layer_,   unitree_home_button_on_axis_,   unitree_home_button_on_dpad_);
            clear_binding(low_level_ctrl_button_, low_level_ctrl_button_on_layer_, low_level_ctrl_button_on_axis_, low_level_ctrl_button_on_dpad_);
            clear_binding(high_level_ctrl_button_,high_level_ctrl_button_on_layer_,high_level_ctrl_button_on_axis_,high_level_ctrl_button_on_dpad_);

            // Publish the sanitized passthrough
            this->GetPublisher<sensor_msgs::msg::Joy>(this->pub_joy_passthrough_key_)->publish(passthrough_msg);
        }

        bool recognizeButton(int btn) {
            return BUTTON_NAMES.count(static_cast<ButtonMap>(btn)) > 0;
        }

        /**
         * Read one FSM binding through its spurious-press filter.
         *
         * STATEFUL for axis bindings -- call it exactly once per binding per received
         * message (see the note at the top of UpdateXHat). Digital and D-pad bindings are
         * passed straight to getButton(): joy_node emits a single message for a digital
         * press, so requiring consecutive samples there would reject short real presses.
         */
        bool getFilteredButton(const sensor_msgs::msg::Joy& msg, AxisPressFilter& filt, int btn,
                               bool on_layer, bool on_axis, bool on_dpad, const char* action) {
            if (!on_axis) {
                return getButton(msg, btn, on_layer, on_axis, on_dpad);
            }

            // With the layer released the binding is not addressable at all, so feed the
            // filter a RELEASED sample (triggers rest at +1.0) rather than the live axis.
            // Otherwise a trigger already held down would arrive pre-confirmed and fire on
            // the very sample the layer button closes.
            float value = 1.0f;
            int code    = btn;
            if (!on_layer ||
                getButton(msg, layer_button_, false, layer_button_on_axis_, layer_button_on_dpad_)) {
                if (on_layer) code -= LAYER_OFFSET;
                value = getAxisValue(msg, code - AXIS_OFFSET);
            }

            const bool pressed = filt.Update(value);

            // Report every attempt the UNFILTERED code would have acted on and this one
            // did not. Unthrottled on purpose: these are rare (11 across 76 recorded runs)
            // and each one is either a caught accident or the operator being told their
            // pull was too soft -- neither is safe to silently drop. The suffix says which
            // it was, so a filter that turns out to be too strict is visible immediately.
            if (filt.last_reject() != AxisPressFilter::Reject::NONE) {
                std::string why;
                if (filt.last_reject() == AxisPressFilter::Reject::UNCONFIRMED) {
                    why = "deep enough but held for fewer than "
                        + std::to_string(filt.confirm_samples()) + " samples";
                } else {
                    why = "only reached " + std::to_string(filt.last_peak())
                        + ", needs " + std::to_string(filt.press_at());
                }
                RCLCPP_WARN_STREAM(this->get_logger(),
                    "[UnitreeJoystick] IGNORED a spurious " << action << " press on "
                    << BUTTON_NAMES.at(static_cast<ButtonMap>(btn)) << ": " << why
                    << ". Pull firmly. (" << filt.rejected() << " ignored / "
                    << filt.accepted() << " accepted this session)");
            }
            return pressed;
        }

        bool getButton(const sensor_msgs::msg::Joy& msg, int btn, bool on_layer, bool on_axis, bool on_dpad){
            if (on_layer) {
                if (!getButton(msg, layer_button_, false, layer_button_on_axis_, layer_button_on_dpad_)) {
                    return false;
                }
                btn -= LAYER_OFFSET;
            }
            if (on_axis) {
                return getAxisValue(msg, btn - AXIS_OFFSET) < axis_threshold_;
            }
            if (on_dpad) {
                int sgn = 1;
                if (btn == static_cast<int>(ButtonMap::DR) || btn == static_cast<int>(ButtonMap::DD)) {
                    btn -= NEG_OFFSET;
                    sgn = -1;
                }
                return msg.axes[btn - AXIS_OFFSET] * sgn > 0.5;
            }
            return msg.buttons[btn];
        }

        float getAxisValue(const sensor_msgs::msg::Joy& msg, int ax) {
            return msg.axes[ax];
        }

        bool isOnLayer(int btn) {
            return btn == static_cast<int>(ButtonMap::X1) || btn == static_cast<int>(ButtonMap::Y1) || btn == static_cast<int>(ButtonMap::A1) || btn == static_cast<int>(ButtonMap::B1) || btn == static_cast<int>(ButtonMap::DL1) || btn == static_cast<int>(ButtonMap::DR1) || btn == static_cast<int>(ButtonMap::DU1) || btn == static_cast<int>(ButtonMap::DD1);
        }

        bool isOnAxis(int btn) {
            return btn == static_cast<int>(ButtonMap::LT) || btn == static_cast<int>(ButtonMap::RT) || btn == static_cast<int>(ButtonMap::LX) || btn == static_cast<int>(ButtonMap::LY) || btn == static_cast<int>(ButtonMap::RX) || btn == static_cast<int>(ButtonMap::RY);
        }

        bool isOnDpad(int btn) {
            return btn == static_cast<int>(ButtonMap::DL) || btn == static_cast<int>(ButtonMap::DR) || btn == static_cast<int>(ButtonMap::DU) || btn == static_cast<int>(ButtonMap::DD) || btn == static_cast<int>(ButtonMap::DL1) || btn == static_cast<int>(ButtonMap::DR1) || btn == static_cast<int>(ButtonMap::DU1) || btn == static_cast<int>(ButtonMap::DD1);
        }

        vel_cmd_msg ComputeControl() override {
            // This callback is not used. We send the high level control in the callback,
            // for safety if the remote is disconnected
            vel_cmd_msg msg;
            return msg;
        };

        // Publisher key
        const std::string pub_exec_fsm_key_ = "pub_exec_fsm";
        const std::string pub_joy_passthrough_key_ = "pub_joy_passthrough";

        // Rate-limits the "sent Execution FSM Command" log (publish stays un-gated).
        FsmLogDebounce fsm_log_debounce_;

        // Hold velocity bounds
        float v_x_scale_;
        float v_y_scale_; 
        float w_z_scale_;
        float axis_threshold_;

        // Spurious-press filtering for FSM bindings on an analog axis (joy_debounce.h).
        float axis_press_threshold_;
        float axis_release_threshold_;
        int axis_confirm_samples_;
        AxisPressFilter estop_filter_;
        AxisPressFilter damping_filter_;
        AxisPressFilter user_pose_filter_;
        AxisPressFilter unitree_home_filter_;
        AxisPressFilter low_level_ctrl_filter_;
        AxisPressFilter high_level_ctrl_filter_;

        // Hold button locations for execution fsm
        int damping_button_;
        int unitree_home_button_;
        int user_pose_button_;
        int low_level_ctrl_button_;
        int high_level_ctrl_button_;
        int estop_;
        int layer_button_;
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
        bool layer_button_on_axis_;
        bool estop_on_dpad_;
        int menu_button_;
        bool menu_button_on_layer_;
        bool menu_button_on_axis_;
        bool menu_button_on_dpad_;

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