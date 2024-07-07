#include "rclcpp/rclcpp.hpp"

#include "obelisk_controller.h"
#include "obelisk_ros_utils.h"

class PositionSetpointController : public obelisk::ObeliskController<obelisk_control_msgs::msg::PositionSetpoint,
                                                                     obelisk_estimator_msgs::msg::EstimatedState> {
  public:
    PositionSetpointController(const std::string& name)
        : obelisk::ObeliskController<obelisk_control_msgs::msg::PositionSetpoint,
                                     obelisk_estimator_msgs::msg::EstimatedState>(name) {}

  protected:
    void UpdateXHat(__attribute__((unused)) const obelisk_estimator_msgs::msg::EstimatedState& msg) override {}

    obelisk_control_msgs::msg::PositionSetpoint ComputeControl() override {
        obelisk_control_msgs::msg::PositionSetpoint msg;

        msg.u.clear();
        rclcpp::Time time = this->get_clock()->now();
        double time_sec   = time.seconds();

        msg.u.emplace_back(sin(time_sec));

        this->GetPublisher<obelisk_control_msgs::msg::PositionSetpoint>(this->ctrl_key_)->publish(msg);

        return msg;
    };
};

int main(int argc, char* argv[]) {
    obelisk::utils::SpinObelisk<PositionSetpointController, rclcpp::executors::MultiThreadedExecutor>(
        argc, argv, "position_setpoint_controller");
}
