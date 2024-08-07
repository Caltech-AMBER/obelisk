#include "rclcpp/rclcpp.hpp"

#include "obelisk_estimator.h"
#include "obelisk_ros_utils.h"

namespace obelisk {
    using leap_estimator_msg = obelisk_estimator_msgs::msg::EstimatedState;

    /**
     * @brief State estimator that uses joint encoders to estimate the state of the Leap hand
     */
    class LeapExampleStateEstimator : public obelisk::ObeliskEstimator<leap_estimator_msg> {
      public:
        LeapExampleStateEstimator(const std::string& name) : obelisk::ObeliskEstimator<leap_estimator_msg>(name) {

            this->RegisterObkSubscription<obelisk_sensor_msgs::msg::ObkJointEncoders>(
                "sub_sensor_setting", "sub_sensor",
                std::bind(&LeapExampleStateEstimator::JointEncoderCallback, this, std::placeholders::_1));
        }

      protected:
        void JointEncoderCallback(const obelisk_sensor_msgs::msg::ObkJointEncoders& msg) {
            joint_encoders_ = msg.joint_pos;
        }

        leap_estimator_msg ComputeStateEstimate() override {
            leap_estimator_msg msg;

            msg.base_link_name = "palm";
            msg.q_joints       = joint_encoders_;
            msg.joint_names    = {"if_mcp", "if_rot", "if_pip", "if_dip", "mf_mcp", "mf_rot", "mf_pip", "mf_dip",
                                  "rf_mcp", "rf_rot", "rf_pip", "rf_dip", "th_cmc", "th_axl", "th_mcp", "th_ipl"};
            this->GetPublisher<leap_estimator_msg>(this->est_pub_key_)->publish(msg);
            return msg;
        };

      private:
        std::vector<double> joint_encoders_;
    };
} // namespace obelisk
