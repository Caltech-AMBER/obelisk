#include <cstdlib>
#include <filesystem>
#include <fstream>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joy_feedback.hpp"

#include "obelisk_controller.h"
#include "obelisk_ros_utils.h"

class PositionSetpointController : public obelisk::ObeliskController<obelisk_control_msgs::msg::PDFeedForward,
                                                                     obelisk_estimator_msgs::msg::EstimatedState> {
  public:
    PositionSetpointController(const std::string& name)
        : obelisk::ObeliskController<obelisk_control_msgs::msg::PDFeedForward,
                                     obelisk_estimator_msgs::msg::EstimatedState>(name) {
        // Example of params_path
        std::string file_string = this->get_parameter("params_path").as_string();
        std::string obk_root    = std::getenv("OBELISK_ROOT");
        std::filesystem::path file_path(obk_root);
        file_path += file_string;

        RCLCPP_INFO_STREAM(this->get_logger(), "File path: " << file_path);

        if (!std::filesystem::exists(file_path)) {
            throw std::runtime_error("file path provided is invalid!");
        }

        std::ifstream params_file(file_path);
        std::string contents;
        params_file >> contents;
        amplitude_ = std::stof(contents);

        // example of setting other configurable params in the node
        this->declare_parameter<std::string>("test_param", "default_value");
        std::string test_param_value;
        this->get_parameter("test_param", test_param_value);
        RCLCPP_INFO_STREAM(this->get_logger(), "test_param: " << test_param_value);

        // Joystick example
        // ----- Joystick Subscriber ----- //
        this->RegisterObkSubscription<sensor_msgs::msg::Joy>(
            "joystick_sub_setting", "joystick_sub",
            std::bind(&PositionSetpointController::JoystickCallback, this, std::placeholders::_1));

        // ----- Joystick Publisher ----- //
        this->RegisterObkPublisher<sensor_msgs::msg::JoyFeedback>("joystick_feedback_setting", "joystick_pub");
    }

  protected:
    void UpdateXHat(__attribute__((unused)) const obelisk_estimator_msgs::msg::EstimatedState& msg) override {}

    obelisk_control_msgs::msg::PDFeedForward ComputeControl() override {
        obelisk_control_msgs::msg::PDFeedForward msg;

        msg.u_mujoco.clear();
        msg.pos_target.clear();
        rclcpp::Time time = this->get_clock()->now();
        double time_sec   = time.seconds();

        msg.u_mujoco.emplace_back(amplitude_ * sin(time_sec));
        msg.pos_target.emplace_back(amplitude_ * sin(time_sec));
        msg.kp.clear();
        msg.kp.emplace_back(10000.0);  // Example gain

        msg.joint_names.clear();
        msg.joint_names.emplace_back("joint1");  // Example joint name

        if (msg.u_mujoco.size() != msg.joint_names.size()) {
            throw std::runtime_error("The size of the control message does not match the number of joint names!");
        }
        // std::cerr << "Publisher ctrl size: " << msg.u_mujoco.size() << std::endl;
        // std::cerr << "Publisher joint names size: " << msg.joint_names.size() << std::endl;

        this->GetPublisher<obelisk_control_msgs::msg::PDFeedForward>(this->ctrl_key_)->publish(msg);

        return msg;
    };

    void JoystickCallback(const sensor_msgs::msg::Joy& msg) {
        RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Joystick message received!");
        if (msg.axes.size() != 8) {
            RCLCPP_ERROR_STREAM_ONCE(this->get_logger(),
                                     "The only supported joy stick for this demo is a xbox remote with 8 axis!");
        } else {
            if (msg.axes[7] == 1) {
                amplitude_ += 0.1;
            } else if (msg.axes[7] == -1) {
                amplitude_ -= 0.1;
            }

            if (msg.buttons[0] == 1) {
                RCLCPP_INFO_STREAM(this->get_logger(), "Press (A) to print this help message. Press the up and down on "
                                                       "the d-pad to adjust the amplitude. Current amplitude: "
                                                           << amplitude_);
            }
        }

        // Rumble the joystick if the amplitude is too large
        if (amplitude_ > 1.1) {
            sensor_msgs::msg::JoyFeedback joy_feedback;
            joy_feedback.type      = sensor_msgs::msg::JoyFeedback::TYPE_RUMBLE;
            joy_feedback.id        = 0;
            joy_feedback.intensity = 0.75;
            this->GetPublisher<sensor_msgs::msg::JoyFeedback>("joystick_pub")->publish(joy_feedback);
        }
    }

    float amplitude_;
};
