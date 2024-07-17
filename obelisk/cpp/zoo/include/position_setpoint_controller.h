#include <cstdlib>
#include <filesystem>
#include <fstream>

#include "rclcpp/rclcpp.hpp"

#include "obelisk_controller.h"
#include "obelisk_ros_utils.h"

class PositionSetpointController : public obelisk::ObeliskController<obelisk_control_msgs::msg::PositionSetpoint,
                                                                     obelisk_estimator_msgs::msg::EstimatedState> {
  public:
    PositionSetpointController(const std::string& name)
        : obelisk::ObeliskController<obelisk_control_msgs::msg::PositionSetpoint,
                                     obelisk_estimator_msgs::msg::EstimatedState>(name) {
        std::string file_string = this->get_parameter("params_path").as_string();
        std::string obk_root    = std::getenv("OBELISK_ROOT");
        std::filesystem::path file_path(obk_root);
        file_path += file_string;

        RCLCPP_WARN_STREAM(this->get_logger(), "File path: " << file_path);

        if (!std::filesystem::exists(file_path)) {
            throw std::runtime_error("file path provided is invalid!");
        }

        std::ifstream params_file(file_path);
        std::string contents;
        params_file >> contents;
        amplitude_ = std::stof(contents);
    }

  protected:
    void UpdateXHat(__attribute__((unused)) const obelisk_estimator_msgs::msg::EstimatedState& msg) override {}

    obelisk_control_msgs::msg::PositionSetpoint ComputeControl() override {
        obelisk_control_msgs::msg::PositionSetpoint msg;

        msg.u.clear();
        rclcpp::Time time = this->get_clock()->now();
        double time_sec   = time.seconds();

        msg.u.emplace_back(amplitude_ * sin(time_sec));

        this->GetPublisher<obelisk_control_msgs::msg::PositionSetpoint>(this->ctrl_key_)->publish(msg);

        return msg;
    };

    float amplitude_;
};
