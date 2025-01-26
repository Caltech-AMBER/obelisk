#include "sensor_msgs/msg/joy.hpp"
#include "rclcpp/rclcpp.hpp"

#include "obelisk_controller.h"
#include "obelisk_ros_utils.h"

namespace obelisk {
    using unitree_controller_msg = obelisk_control_msgs::msg::PDFeedForward;
    using unitree_estimator_msg  = obelisk_estimator_msgs::msg::EstimatedState;

    class UnitreeExampleController : public ObeliskController<unitree_controller_msg, unitree_estimator_msg> {
      public:
        UnitreeExampleController(const std::string& name)
            : ObeliskController<unitree_controller_msg, unitree_estimator_msg>(name), joint_idx_(0) {
            // ----- Joystick Subscriber ----- //
            this->RegisterObkSubscription<sensor_msgs::msg::Joy>(
                        "joystick_sub_setting", "joystick_sub",
                        std::bind(&UnitreeExampleController::JoystickCallback, this, std::placeholders::_1));

            RCLCPP_INFO_STREAM(this->get_logger(), "New Joint Index: " << joint_idx_ << ", Expected Joint: " << JOINT_NAMES[joint_idx_]);
        }

      protected:
        void UpdateXHat(__attribute__((unused)) const unitree_estimator_msg& msg) override {}

        unitree_controller_msg ComputeControl() override {
            unitree_controller_msg msg;

            msg.u_mujoco.clear();
            msg.pos_target.clear();
            msg.joint_names.clear();
            msg.kp.clear();
            msg.kd.clear();
            double time_sec = this->get_clock()->now().seconds();

            // TODO With hands we have 43
            float offset = 0;

            for (int i = 0; i < MOTOR_NUM; i++) {
                if (i == joint_idx_) {
                    if (JOINT_NAMES[i].find("calf") != std::string::npos) {
                        offset = -1.5;
                    } else {
                        offset = 0;
                    }
                    msg.u_mujoco.emplace_back(amplitude_ * sin(time_sec) + offset);
                    msg.pos_target.emplace_back(amplitude_ * sin(time_sec) + offset);
                    msg.kp.push_back(20);
                    msg.kd.push_back(2);
                } else {
                    msg.u_mujoco.emplace_back(0);
                    msg.pos_target.emplace_back(0);
                    msg.kp.push_back(10);
                    msg.kd.push_back(1);
                }

                msg.vel_target.emplace_back(0);
                msg.feed_forward.emplace_back(0);
            }

            for (int i = 0; i < 2*MOTOR_NUM; i++) {
                msg.u_mujoco.emplace_back(0);
            }

            for (int i = 0; i < MOTOR_NUM; i++) {
                msg.joint_names.push_back(JOINT_NAMES[i]);
            }

            this->GetPublisher<unitree_controller_msg>(this->ctrl_key_)->publish(msg);

            return msg;
        };

        void JoystickCallback(const sensor_msgs::msg::Joy& msg) {
            // ----- Axes ----- //
            // constexpr int DPAD_VERTICAL = 7;
            // constexpr int DPAD_HORIZONTAL = 6;

            // constexpr int RIGHT_JOY_VERT = 4;
            // constexpr int RIGHT_JOY_HORZ = 3;

            // constexpr int LEFT_JOY_VERT = 1;
            // constexpr int LEFT_JOY_HORZ = 0;

            // constexpr int LEFT_TRIGGER = 2;
            // constexpr int RIGHT_TRIGGER = 5;

            // ----- Buttons ----- //
            // constexpr int A = 0;
            // constexpr int B = 1;
            constexpr int X = 2;
            // constexpr int Y = 3;

            // constexpr int LEFT_BUMPER = 4;
            // constexpr int RIGHT_BUMPER = 5;

            // constexpr int MENU = 7;
            // constexpr int SQUARES = 6;

            // static rclcpp::Time last_menu_press = this->now();
            // static rclcpp::Time last_A_press = this->now();
            static rclcpp::Time last_X_press = this->now();
            // static rclcpp::Time last_B_press = this->now();
            // static rclcpp::Time last_target_update = this->now();

            if (msg.buttons[X] && (this->now() - last_X_press).seconds() > 1) {
                
                joint_idx_++;
                joint_idx_ = joint_idx_ % MOTOR_NUM;

                RCLCPP_INFO_STREAM(this->get_logger(), "New Joint Index: " << joint_idx_ << ", Expected Joint: " << JOINT_NAMES[joint_idx_]);

                last_X_press = this->now();
            }
        }

        float amplitude_ = 0.2;

        int joint_idx_;

        // static constexpr int MOTOR_NUM = 29;
        // const std::array<std::string, MOTOR_NUM> JOINT_NAMES = {
        //     "left_hip_pitch_joint",
        //     "left_hip_roll_joint",
        //     "left_hip_yaw_joint",
        //     "left_knee_joint",
        //     "left_ankle_pitch_joint",
        //     "left_ankle_roll_joint",
        //     "right_hip_pitch_joint",
        //     "right_hip_roll_joint",
        //     "right_hip_yaw_joint",
        //     "right_knee_joint",
        //     "right_ankle_pitch_joint",
        //     "right_ankle_roll_joint",
        //     "waist_yaw_joint",
        //     "waist_roll_joint",
        //     "waist_pitch_joint",
        //     "left_shoulder_pitch_joint",
        //     "left_shoulder_roll_joint",
        //     "left_shoulder_yaw_joint",
        //     "left_elbow_joint",
        //     "left_wrist_roll_joint",
        //     "left_wrist_pitch_joint",
        //     "left_wrist_yaw_joint",
        //     "right_shoulder_pitch_joint",
        //     "right_shoulder_roll_joint",
        //     "right_shoulder_yaw_joint",
        //     "right_elbow_joint",
        //     "right_wrist_roll_joint",
        //     "right_wrist_pitch_joint",
        //     "right_wrist_yaw_joint",
        // };

        // TODO: identify robot from context?
        static constexpr int MOTOR_NUM = 12;
        const std::array<std::string, MOTOR_NUM> JOINT_NAMES = {
            "FR_hip_joint",
            "FR_thigh_joint",
            "FR_calf_joint",
            "FL_hip_joint",
            "FL_thigh_joint",
            "FL_calf_joint",
            "RR_hip_joint",
            "RR_thigh_joint",
            "RR_calf_joint",
            "RL_hip_joint",
            "RL_thigh_joint",
            "RL_calf_joint"
        };
    };
} // namespace obelisk
