#include "rclcpp/rclcpp.hpp"

#include "obelisk_estimator.h"
#include "obelisk_ros_utls.h"

class JointEncodersPassthrough : public obelisk::ObeliskEstimator<obelisk_estimator_msgs::msg::EstimatedState> {
  public:
    JointEncodersPassthrough(const std::string& name)
        : obelisk::ObeliskEstimator<obelisk_estimator_msgs::msg::EstimatedState>(name) {

        this->RegisterSubscription<obelisk_sensor_msgs::msg::JointEncoders>(
            "sub_sensor_setting", "sub_sensor",
            std::bind(&JointEncodersPassthrough::JointEncoderCallback, this, std::placeholders::_1));
    }

  protected:
    void JointEncoderCallback(const obelisk_sensor_msgs::msg::JointEncoders& msg) { joint_encoders_ = msg.y; }

    obelisk_estimator_msgs::msg::EstimatedState ComputeStateEstimate() override {
        obelisk_estimator_msgs::msg::EstimatedState msg;

        msg.x_hat = joint_encoders_;

        this->GetPublisher<obelisk_estimator_msgs::msg::EstimatedState>(this->est_pub_key_)->publish(msg);

        return msg;
    };

  private:
    std::vector<double> joint_encoders_;
};

int main(int argc, char* argv[]) {
    obelisk::utils::SpinObelisk<JointEncodersPassthrough, rclcpp::executors::MultiThreadedExecutor>(
        argc, argv, "passthrough_estimator");
}
