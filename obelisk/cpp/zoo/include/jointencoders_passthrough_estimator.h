#include "rclcpp/rclcpp.hpp"

#include "obelisk_estimator.h"
#include "obelisk_ros_utils.h"

class JointEncodersPassthroughEstimator
    : public obelisk::ObeliskEstimator<obelisk_estimator_msgs::msg::EstimatedState> {
  public:
    JointEncodersPassthroughEstimator(const std::string& name)
        : obelisk::ObeliskEstimator<obelisk_estimator_msgs::msg::EstimatedState>(name) {

        this->RegisterObkSubscription<obelisk_sensor_msgs::msg::ObkJointEncoders>(
            "sub_sensor_setting", "sub_sensor",
            std::bind(&JointEncodersPassthroughEstimator::JointEncoderCallback, this, std::placeholders::_1));
    }

  protected:
    void JointEncoderCallback(const obelisk_sensor_msgs::msg::ObkJointEncoders& msg) {
        joint_encoders_ = msg.joint_pos;
    }

    obelisk_estimator_msgs::msg::EstimatedState ComputeStateEstimate() override {
        obelisk_estimator_msgs::msg::EstimatedState msg;

        msg.x_hat          = joint_encoders_;
        msg.q_joints       = joint_encoders_;
        msg.joint_names    = {"joint1"};
        msg.base_link_name = "link0";

        this->GetPublisher<obelisk_estimator_msgs::msg::EstimatedState>(this->est_pub_key_)->publish(msg);

        return msg;
    };

  private:
    std::vector<double> joint_encoders_;
};
