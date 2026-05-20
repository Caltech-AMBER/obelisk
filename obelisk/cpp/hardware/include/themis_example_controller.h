#pragma once

#include <cmath>
#include <map>
#include <string>
#include <vector>

#include "obelisk_controller.h"
#include "obelisk_ros_utils.h"
#include "obelisk_control_msgs/msg/pd_feed_forward.hpp"
#include "obelisk_estimator_msgs/msg/estimated_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "rclcpp/rclcpp.hpp"

namespace obelisk {

    using themis_example_ctrl_msg = obelisk_control_msgs::msg::PDFeedForward;
    using themis_example_est_msg  = obelisk_estimator_msgs::msg::EstimatedState;

    /**
     * @brief Stub controller that sweeps one THEMIS joint sinusoidally at a time.
     *
     * Holds all 28 joints at their home pose with low gains (kp=10, kd=1). One
     * joint, selected by `joint_idx_`, is driven with `amplitude * sin(t) +
     * default` at higher gains (kp=20, kd=2). Joystick "X" button advances
     * `joint_idx_` (1 s debounce).
     *
     * Mirrors obelisk/cpp/hardware/include/unitree_example_controller.h. Useful
     * as an integration smoke test for ThemisInterface — exercises the full
     * PDFeedForward → FULL_BODY_CMD path without any RL code.
     */
    class ThemisExampleController : public ObeliskController<themis_example_ctrl_msg, themis_example_est_msg> {
      public:
        explicit ThemisExampleController(const std::string& name)
            : ObeliskController<themis_example_ctrl_msg, themis_example_est_msg>(name) {
            this->RegisterObkSubscription<sensor_msgs::msg::Joy>(
                "joystick_sub",
                std::bind(&ThemisExampleController::JoystickCallback, this, std::placeholders::_1));

            RCLCPP_INFO_STREAM(this->get_logger(), "New Joint Index: " << joint_idx_
                                                       << ", Expected Joint: " << JOINT_NAMES[joint_idx_]);
        }

      protected:
        void UpdateXHat(__attribute__((unused)) const themis_example_est_msg& msg) override {}

        themis_example_ctrl_msg ComputeControl() override {
            themis_example_ctrl_msg msg;
            msg.pos_target.clear();
            msg.vel_target.clear();
            msg.feed_forward.clear();
            msg.joint_names.clear();
            msg.kp.clear();
            msg.kd.clear();
            msg.u_mujoco.clear();

            const double t = this->get_clock()->now().seconds();
            for (size_t i = 0; i < NUM_JOINTS; ++i) {
                const double default_q = JOINT_DEFAULTS.at(JOINT_NAMES[i]);
                const double q = (i == joint_idx_) ? AMPLITUDE * std::sin(t) + default_q : default_q;
                msg.pos_target.push_back(q);
                msg.vel_target.push_back(0.0);
                msg.feed_forward.push_back(0.0);
                msg.joint_names.push_back(JOINT_NAMES[i]);
                msg.kp.push_back((i == joint_idx_) ? 20.0 : 10.0);
                msg.kd.push_back((i == joint_idx_) ? 2.0  : 1.0);
                // u_mujoco carries the per-actuator ctrl value for sim. The MJCF uses implicit-PD
                // actuators (tau = kp*(ctrl-q) - kd*q̇), so ctrl is a position setpoint.
                msg.u_mujoco.push_back(q);
            }

            this->GetPublisher<themis_example_ctrl_msg>(this->ctrl_key_)->publish(msg);
            return msg;
        }

        void JoystickCallback(const sensor_msgs::msg::Joy& msg) {
            constexpr int X = 3;  // (typical Xbox layout — same convention as unitree_example_controller)
            static rclcpp::Time last_x_press = this->now();
            if (msg.buttons.size() > static_cast<size_t>(X)
                && msg.buttons[X]
                && (this->now() - last_x_press).seconds() > 1.0) {
                joint_idx_ = (joint_idx_ + 1) % NUM_JOINTS;
                RCLCPP_INFO_STREAM(this->get_logger(),
                    "New Joint Index: " << joint_idx_ << ", Expected Joint: " << JOINT_NAMES[joint_idx_]);
                last_x_press = this->now();
            }
        }

      private:
        static constexpr double AMPLITUDE  = 0.2;
        static constexpr size_t NUM_JOINTS = 28;
        size_t joint_idx_ = 0;

        // Canonical THEMIS joint order: [right_leg(6), left_leg(6), right_arm(7), left_arm(7), head(2)].
        // Matches ThemisInterface::ThemisJointNames().
        inline static const std::vector<std::string> JOINT_NAMES = {
            "HIP_YAW_R", "HIP_ROLL_R", "HIP_PITCH_R", "KNEE_PITCH_R", "ANKLE_PITCH_R", "ANKLE_ROLL_R",
            "HIP_YAW_L", "HIP_ROLL_L", "HIP_PITCH_L", "KNEE_PITCH_L", "ANKLE_PITCH_L", "ANKLE_ROLL_L",
            "SHOULDER_PITCH_R", "SHOULDER_ROLL_R", "SHOULDER_YAW_R", "ELBOW_PITCH_R", "ELBOW_YAW_R",
            "WRIST_PITCH_R", "WRIST_YAW_R",
            "SHOULDER_PITCH_L", "SHOULDER_ROLL_L", "SHOULDER_YAW_L", "ELBOW_PITCH_L", "ELBOW_YAW_L",
            "WRIST_PITCH_L", "WRIST_YAW_L",
            "HEAD_YAW", "HEAD_PITCH",
        };

        // Home pose in POLICY frame (= themis_rl's training MJCF frame, which our themis_description
        // MJCF also uses).  Values copied verbatim from themis_rl/sim2real/config.py DEFAULT_JOINT_POS.
        //
        // NOTE on hardware deployment: AOS expects HW frame (URDF convention). The frame conversion
        // q_hw = sign*(q_pol - offset) from themis_rl/sim2real/joint_mapping.py needs to be applied
        // by themis_obk_server.py before passing setpoints to wbc_api.  TODO until that's wired.
        inline static const std::map<std::string, double> JOINT_DEFAULTS = {
            {"HIP_YAW_R",        0.000},  {"HIP_ROLL_R",       0.000},  {"HIP_PITCH_R",     -0.270},
            {"KNEE_PITCH_R",     0.570},  {"ANKLE_PITCH_R",   -0.300},  {"ANKLE_ROLL_R",     0.000},
            {"HIP_YAW_L",        0.000},  {"HIP_ROLL_L",       0.000},  {"HIP_PITCH_L",     -0.270},
            {"KNEE_PITCH_L",     0.570},  {"ANKLE_PITCH_L",   -0.300},  {"ANKLE_ROLL_L",     0.000},
            {"SHOULDER_PITCH_R", 0.200},  {"SHOULDER_ROLL_R", -0.300},  {"SHOULDER_YAW_R",   0.000},
            {"ELBOW_PITCH_R",    1.300},  {"ELBOW_YAW_R",      0.000},
            {"WRIST_PITCH_R",    0.000},  {"WRIST_YAW_R",      0.000},
            {"SHOULDER_PITCH_L", 0.200},  {"SHOULDER_ROLL_L",  0.300},  {"SHOULDER_YAW_L",   0.000},
            {"ELBOW_PITCH_L",    1.300},  {"ELBOW_YAW_L",      0.000},
            {"WRIST_PITCH_L",    0.000},  {"WRIST_YAW_L",      0.000},
            {"HEAD_YAW",         0.000},  {"HEAD_PITCH",       0.000},
        };
    };

}  // namespace obelisk
