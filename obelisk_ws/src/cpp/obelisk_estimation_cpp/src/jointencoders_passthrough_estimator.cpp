#include "rclcpp/rclcpp.hpp"

#include "obelisk_estimator.h"
#include "obelisk_ros_utls.h"

class JointEncodersPassthrough : public obelisk::ObeliskEstimator<obelisk_estimator_msgs::msg::EstimatedState> {
  public:
    JointEncodersPassthrough(const std::string& name)
        : obelisk::ObeliskEstimator<obelisk_estimator_msgs::msg::EstimatedState>(name) {
        this->create_subscription<obelisk_sensor_msgs::msg::JointEncoders>(
            "joint_sub", 10, std::bind(&JointEncodersPassthrough::JointEncoderCallback, this, std::placeholders::_1));
    }

  protected:
    void JointEncoderCallback(const obelisk_sensor_msgs::msg::JointEncoders& msg) { joint_encoders_ = msg.y; }

    obelisk_estimator_msgs::msg::EstimatedState ComputeStateEstimate() override {
        obelisk_estimator_msgs::msg::EstimatedState msg;

        msg.x_hat = joint_encoders_;

        this->estimator_publisher_->publish(msg);

        return msg;
    };

  private:
    std::vector<double> joint_encoders_;
};

int main(int argc, char* argv[]) {
    obelisk::utils::SpinObelisk<JointEncodersPassthrough, rclcpp::executors::MultiThreadedExecutor>(
        argc, argv, "passthrough_estimator");
}
