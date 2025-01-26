#include "unitree_interface.h"

// IDL
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>

namespace obelisk {

    using namespace unitree_go::msg::dds_;

    class Go2Interface : public ObeliskUnitreeInterface {
    public:
        Go2Interface(const std::string& node_name)
            : ObeliskUnitreeInterface(node_name) {
            // TODO: Arm on top of quad??

            for (int i = 0; i < GO2_MOTOR_NUM; i++) {
                joint_names_.push_back(GO2_JOINT_NAMES[i]);
            } 

            CMD_TOPIC_ = "rt/lowcmd";
            STATE_TOPIC_ = "rt/lowstate";
            // TODO: Look for ODOM topic

            VerifyParameters();
            CreateUnitreePublishers();
        }
    protected:
        void CreateUnitreePublishers() override {
            // create publisher
            lowcmd_publisher_.reset(new ChannelPublisher<LowCmd_>(CMD_TOPIC_));
            lowcmd_publisher_->InitChannel();

            RCLCPP_INFO_STREAM(this->get_logger(), "Go2 command publishers created!");
        }

        void CreateUnitreeSubscribers() override {
            // create subscriber
            // Need to create after the publishers have been activated
            lowstate_subscriber_.reset(new ChannelSubscriber<LowState_>(STATE_TOPIC_));
            lowstate_subscriber_->InitChannel(std::bind(&Go2Interface::LowStateHandler, this, std::placeholders::_1), 1);

            // Odom state?
        }

        void ApplyControl(const unitree_control_msg& msg) override {
            // Apply control to the robot
            RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Applying control to Unitree robot!");

            // ---------- Verify message ---------- //
            if (msg.joint_names.size() != msg.pos_target.size() || msg.joint_names.size() != msg.vel_target.size() || msg.joint_names.size() != msg.feed_forward.size()) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "joint name size: " << msg.joint_names.size());
                RCLCPP_ERROR_STREAM(this->get_logger(), "position size: " << msg.pos_target.size());
                RCLCPP_ERROR_STREAM(this->get_logger(), "velocity size: " << msg.vel_target.size());
                RCLCPP_ERROR_STREAM(this->get_logger(), "feed forward size: " << msg.feed_forward.size());
                throw std::runtime_error("[UnitreeRobotInterface] Control message sizes are not self consistent!");
            }

            if (msg.kp.size() != msg.kd.size()) {
                throw std::runtime_error("[UnitreeRobotInterface] Control message gains are not self consistent!");
            }

            // Check that every joint name is in the list and that they are all there
            std::vector<bool> joint_used(GO2_MOTOR_NUM, false);
            std::vector<int> joint_mapping(GO2_MOTOR_NUM, -1);    // Maps the j'th input joint to the i'th output joint
            if (msg.joint_names.size() != joint_names_.size()) {
                throw std::runtime_error("[UnitreeRobotInterface] Control message joint name size does not match robot joint name size!");
            }

            for (size_t j = 0; j < GO2_MOTOR_NUM; j++) {
                for (size_t i = 0; i < GO2_MOTOR_NUM; i++) {
                    if (msg.joint_names[j] == joint_names_[i]) {
                        if (joint_used[i]) {
                            throw std::runtime_error("[UnitreeRobotInterface] Control message uses the same joint name twice!");
                        }

                        joint_used[i] = true;
                        joint_mapping[j] = i;
                    }
                }
            }
            for (size_t j = 0; j < GO2_MOTOR_NUM; j++) {
                if (!joint_used[j]) {
                    throw std::runtime_error("[UnitreeRobotInterface] Control message is missing robot joints!");
                }
            }

            bool use_default_gains = true;
            if (msg.joint_names.size() == msg.kp.size()) {
                if (msg.kp.size() != GO2_MOTOR_NUM) {
                    throw std::runtime_error("[UnitreeRobotInterface] Control message gains are not of the right size!");
                }
                use_default_gains = false;
            }

            // ---------- Create Packet ---------- //
            LowCmd_ dds_low_command;

            for (size_t j = 0; j < GO2_MOTOR_NUM; j++) {     // Only go through the non-hand motors
                int i = joint_mapping[j];
                dds_low_command.motor_cmd().at(i).mode() = 1;  // 1:Enable, 0:Disable
                dds_low_command.motor_cmd().at(i).tau() = msg.feed_forward[j];
                dds_low_command.motor_cmd().at(i).q() = msg.pos_target[j];
                dds_low_command.motor_cmd().at(i).dq() = msg.vel_target[j];
                dds_low_command.motor_cmd().at(i).kp() = use_default_gains ? kp_[i] : msg.kp[j];
                dds_low_command.motor_cmd().at(i).kd() = use_default_gains ? kd_[i] : msg.kd[j];
            }

            dds_low_command.crc() = Crc32Core((uint32_t *)&dds_low_command, (sizeof(dds_low_command) >> 2) - 1);
            lowcmd_publisher_->Write(dds_low_command);
        }

        void LowStateHandler(const void *message) {
            LowState_ low_state = *(const LowState_ *)message;
            if (low_state.crc() != Crc32Core((uint32_t *)&low_state, (sizeof(LowState_) >> 2) - 1)) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "[UnitreeRobotInterface] CRC Error");
                return;
            }

            // Joints
            obelisk_sensor_msgs::msg::ObkJointEncoders joint_state;
            joint_state.header.stamp = this->now();
            joint_state.joint_pos.resize(GO2_MOTOR_NUM);
            joint_state.joint_vel.resize(GO2_MOTOR_NUM);
            joint_state.joint_names.resize(GO2_MOTOR_NUM);

            for (size_t i = 0; i < GO2_MOTOR_NUM; ++i) {
                joint_state.joint_names.at(i) = joint_names_[i];
                joint_state.joint_pos.at(i) = low_state.motor_state()[i].q();
                joint_state.joint_vel.at(i) = low_state.motor_state()[i].dq();
                // if (low_state.motor_state()[i].motorstate() && i <= RightAnkleRoll) // TODO: What is this?
                // std::cout << "[ERROR] motor " << i << " with code " << low_state.motor_state()[i].motorstate() << "\n";
            }

            this->GetPublisher<obelisk_sensor_msgs::msg::ObkJointEncoders>("joint_state_pub")->publish(joint_state);

            // IMU
            obelisk_sensor_msgs::msg::ObkImu imu_state;
            imu_state.header.stamp = this->now();
            imu_state.angular_velocity.x = low_state.imu_state().gyroscope()[0];
            imu_state.angular_velocity.y = low_state.imu_state().gyroscope()[1];
            imu_state.angular_velocity.z = low_state.imu_state().gyroscope()[2];

            imu_state.orientation.w = low_state.imu_state().quaternion()[0];
            imu_state.orientation.x = low_state.imu_state().quaternion()[1];
            imu_state.orientation.y = low_state.imu_state().quaternion()[2];
            imu_state.orientation.z = low_state.imu_state().quaternion()[3];

            imu_state.linear_acceleration.x = low_state.imu_state().accelerometer()[0];
            imu_state.linear_acceleration.y= low_state.imu_state().accelerometer()[1];
            imu_state.linear_acceleration.z = low_state.imu_state().accelerometer()[2];

            this->GetPublisher<obelisk_sensor_msgs::msg::ObkImu>("imu_state_pub")->publish(imu_state);

            // TODO: foot forces
        }

        // void OdomHandler(const void *message) {
        //     RCLCPP_INFO_STREAM(this->get_logger(), "IN ODOM");
        //     IMUState_ imu_state = *(const IMUState_ *)message;
        //     // if (imu_state.crc() != Crc32Core((uint32_t *)&imu_state, (sizeof(imu_state) >> 2) - 1)) {
        //     //     RCLCPP_ERROR_STREAM(this->get_logger(), "[UnitreeRobotInterface] CRC Error");
        //     //     return;
        //     // }

        //     // IMU
        //     obelisk_sensor_msgs::msg::ObkImu obk_imu_state;
        //     obk_imu_state.header.stamp = this->now();
        //     obk_imu_state.angular_velocity.x = imu_state.gyroscope()[0];
        //     obk_imu_state.angular_velocity.y = imu_state.gyroscope()[1];
        //     obk_imu_state.angular_velocity.z = imu_state.gyroscope()[2];

        //     obk_imu_state.orientation.w = imu_state.quaternion()[0];
        //     obk_imu_state.orientation.x = imu_state.quaternion()[1];
        //     obk_imu_state.orientation.y = imu_state.quaternion()[2];
        //     obk_imu_state.orientation.z = imu_state.quaternion()[3];

        //     obk_imu_state.linear_acceleration.x = imu_state.accelerometer()[0];
        //     obk_imu_state.linear_acceleration.y= imu_state.accelerometer()[1];
        //     obk_imu_state.linear_acceleration.z = imu_state.accelerometer()[2];

        //     this->GetPublisher<obelisk_sensor_msgs::msg::ObkImu>("odom_state_pub")->publish(obk_imu_state);
        // }

        std::string ODOM_TOPIC_;

        unitree::robot::ChannelPublisherPtr<LowCmd_> lowcmd_publisher_;
        ChannelSubscriberPtr<LowState_> lowstate_subscriber_;
        // ChannelSubscriberPtr<IMUState_> odom_subscriber_;

    private:
        // ---------- Robot Specific ---------- //
        // Go2
        static constexpr int GO2_MOTOR_NUM = 12;
        const std::array<std::string, GO2_MOTOR_NUM> GO2_JOINT_NAMES = {
            "FR_hip_joint",
            "FR_thigh_joint",
            "FR_calf_joint",
            "FL_hip_joint",
            "FL_thigh_joint",
            "FL_calf_joint",
            "RR_hip_joint",
            "RR_thigh_joint",
            "RR_calf_joint",
            "RL_hip_joint",
            "RL_thigh_joint",
            "RL_calf_joint"
        };
    };
}   // namespace obelisk