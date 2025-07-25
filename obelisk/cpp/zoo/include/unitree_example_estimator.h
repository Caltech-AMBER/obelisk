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
            // t_last_update_ = msg.header.stamp.nanosec / std::pow(10, 9); // time since node started
            t_last_update_ = this->now().nanoseconds() / std::pow(10, 9); // time since 1970
            joint_encoders_ = msg.joint_pos;
            joint_vels_ = msg.joint_vel;
            joint_names_ = msg.joint_names;
        }

        obelisk_estimator_msgs::msg::EstimatedState ComputeStateEstimate() override {
            // This is called by a timer specified in the .yaml file.

            // If too many seconds has passed without an update, throw an error.
            double current_time = this->now().nanoseconds() / std::pow(10, 9); // time since 1970
            double t_without_update = current_time - t_last_update_;
            // RCLCPP_INFO_STREAM(this->get_logger(), "t_without_update " << t_without_update);

            if (t_last_update_ != 0
                && t_without_update > MAX_TIME_WITHOUT_UPDATE) {
               throw std::runtime_error(
                "No state estimate received within the last "
                + std::to_string(MAX_TIME_WITHOUT_UPDATE)
                + " seconds. Robot may have crashed."
                );
            }

            // Create the estimated msg
            obelisk_estimator_msgs::msg::EstimatedState msg;

            msg.header.stamp = this->now();
            msg.q_joints = joint_encoders_; // Joint Positions
            msg.v_joints = joint_vels_; // Joint Velocities
            msg.joint_names = joint_names_; // Joint Names
            msg.base_link_name = BASE_LINK_NAME;

            // If there are values stored in the joint encoders, publish
            // the estimated message
            if (joint_encoders_.size() != 0) {
                this->GetPublisher<obelisk_estimator_msgs::msg::EstimatedState>(this->est_pub_key_)->publish(msg);
            }
            return msg;
        };

    private:
        const double MAX_TIME_WITHOUT_UPDATE = 1; // seconds
        const std::string BASE_LINK_NAME = "base_link";

        double t_last_update_; // defaults to 0
        std::vector<double> joint_encoders_;
        std::vector<double> joint_vels_;
        std::vector<std::string> joint_names_;
    };
}   // namespace obelisk