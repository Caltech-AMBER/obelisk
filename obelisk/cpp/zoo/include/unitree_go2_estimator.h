#include "rclcpp/rclcpp.hpp"

#include "obelisk_estimator.h"
#include "obelisk_ros_utils.h"

namespace obelisk {
    class UnitreeGo2Estimator
        : public obelisk::ObeliskEstimator<obelisk_estimator_msgs::msg::EstimatedState> {
    public:
        UnitreeGo2Estimator(const std::string& name)
            : obelisk::ObeliskEstimator<obelisk_estimator_msgs::msg::EstimatedState>(name) {

            this->RegisterObkSubscription<obelisk_sensor_msgs::msg::ObkJointEncoders>(
                "sub_sensor_setting", "sub_sensor",
                std::bind(&UnitreeGo2Estimator::JointEncoderCallback, this, std::placeholders::_1));

            this->RegisterObkSubscription<obelisk_sensor_msgs::msg::ObkImu>(
                "sub_imu_setting", "imu_sensor",
                std::bind(&UnitreeGo2Estimator::TorsoIMUCallback, this, std::placeholders::_1));
        }

    protected:
        void JointEncoderCallback(const obelisk_sensor_msgs::msg::ObkJointEncoders& msg) {
            joint_encoders_ = msg.joint_pos;
            joint_vels_ = msg.joint_vel;
            joint_names_ = msg.joint_names;
        }

        void TorsoIMUCallback(const obelisk_sensor_msgs::msg::ObkImu& msg) {
            pose_ = {0., 0., 0., msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w};
            omega_ = {0., 0., 0., msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z};
        }

        obelisk_estimator_msgs::msg::EstimatedState ComputeStateEstimate() override {
            obelisk_estimator_msgs::msg::EstimatedState msg;

            msg.header.stamp = this->now();
            msg.base_link_name = "torso";
            msg.q_joints       = joint_encoders_;   // Joint Positions
            msg.v_joints       = joint_vels_;        // Joint Velocities
            msg.joint_names    = joint_names_;      // Joint Names
            msg.q_base         = pose_;             // Quaternion
            msg.v_base         = omega_;            // Angular Velocity
            // msg.base_link_name = "link0";

            this->GetPublisher<obelisk_estimator_msgs::msg::EstimatedState>(this->est_pub_key_)->publish(msg);

            return msg;
        };

    private:
        std::vector<double> joint_encoders_;
        std::vector<double> joint_vels_;
        std::vector<double> pose_;
        std::vector<double> omega_;
        std::vector<std::string> joint_names_;
    };
}   // namespace obelisk