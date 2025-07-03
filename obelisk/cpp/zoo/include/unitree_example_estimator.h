#include "rclcpp/rclcpp.hpp"

#include "obelisk_estimator.h"
#include "obelisk_ros_utils.h"

namespace obelisk {
    class UnitreeExampleEstimator
        : public obelisk::ObeliskEstimator<obelisk_estimator_msgs::msg::EstimatedState> {
    public:
        UnitreeExampleEstimator(const std::string& name)
            : obelisk::ObeliskEstimator<obelisk_estimator_msgs::msg::EstimatedState>(name) {
            this->RegisterObkSubscription<obelisk_sensor_msgs::msg::ObkJointEncoders>(
                "sub_sensor_setting", "sub_sensor",
                std::bind(&UnitreeExampleEstimator::JointEncoderCallback, this, std::placeholders::_1));
        }

    protected:
        void JointEncoderCallback(const obelisk_sensor_msgs::msg::ObkJointEncoders& msg) {
            joint_encoders_ = msg.joint_pos;
            joint_vels_ = msg.joint_vel;
            joint_names_ = msg.joint_names;
        }

        obelisk_estimator_msgs::msg::EstimatedState ComputeStateEstimate() override {
            obelisk_estimator_msgs::msg::EstimatedState msg;

            msg.header.stamp = this->now();
            msg.q_joints       = joint_encoders_;   // Joint Positions
            msg.v_joints       = joint_vels_;        // Joint Velocities
            msg.joint_names    = joint_names_;      // Joint Names
            msg.base_link_name = "base_link";

            this->GetPublisher<obelisk_estimator_msgs::msg::EstimatedState>(this->est_pub_key_)->publish(msg);

            return msg;
        };

    private:
        std::vector<double> joint_encoders_;
        std::vector<double> joint_vels_;
        std::vector<std::string> joint_names_;
    };
}   // namespace obelisk