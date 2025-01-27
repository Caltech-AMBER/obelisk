#include "unitree_interface.h"

// Sport Client
#include <unitree/robot/go2/sport/sport_client.hpp>

// IDL
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>

namespace obelisk {

    using namespace unitree_go::msg::dds_;

    class Go2Interface : public ObeliskUnitreeInterface {
    public:
        Go2Interface(const std::string& node_name)
            : ObeliskUnitreeInterface(node_name) {
            num_motors_ = GO2_MOTOR_NUM;

            for (size_t i = 0; i < num_motors_; i++) {
                joint_names_.push_back(GO2_JOINT_NAMES[i]);
            } 

            CMD_TOPIC_ = "rt/lowcmd";
            STATE_TOPIC_ = "rt/lowstate";
            // TODO: Look for ODOM topic

            VerifyParameters();
            CreateUnitreePublishers();

            sport_client_.SetTimeout(10.0f);
            sport_client_.Init();
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
            if (exec_fsm_state_ != ExecFSMState::LOW_LEVEL_CTRL) {
                RCLCPP_DEBUG_STREAM(this->get_logger(), "Ignoring low level control commands!");
                return;
            }
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
            std::vector<bool> joint_used(num_motors_, false);
            std::vector<int> joint_mapping(num_motors_, -1);    // Maps the j'th input joint to the i'th output joint
            if (msg.joint_names.size() != joint_names_.size()) {
                throw std::runtime_error("[UnitreeRobotInterface] Control message joint name size does not match robot joint name size!");
            }

            for (size_t j = 0; j < num_motors_; j++) {
                for (size_t i = 0; i < num_motors_; i++) {
                    if (msg.joint_names[j] == joint_names_[i]) {
                        if (joint_used[i]) {
                            throw std::runtime_error("[UnitreeRobotInterface] Control message uses the same joint name twice!");
                        }

                        joint_used[i] = true;
                        joint_mapping[j] = i;
                    }
                }
            }
            for (size_t j = 0; j < num_motors_; j++) {
                if (!joint_used[j]) {
                    throw std::runtime_error("[UnitreeRobotInterface] Control message is missing robot joints!");
                }
            }

            bool use_default_gains = true;
            if (msg.joint_names.size() == msg.kp.size()) {
                if (msg.kp.size() != num_motors_) {
                    throw std::runtime_error("[UnitreeRobotInterface] Control message gains are not of the right size!");
                }
                use_default_gains = false;
            }

            // ---------- Create Packet ---------- //
            LowCmd_ dds_low_command;

            for (size_t j = 0; j < num_motors_; j++) {     // Only go through the non-hand motors
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
            joint_state.joint_pos.resize(num_motors_);
            joint_state.joint_vel.resize(num_motors_);
            joint_state.joint_names.resize(num_motors_);

            for (size_t i = 0; i < num_motors_; ++i) {
                joint_state.joint_names.at(i) = joint_names_[i];
                joint_state.joint_pos.at(i) = low_state.motor_state()[i].q();
                joint_state.joint_vel.at(i) = low_state.motor_state()[i].dq();
                {
                    std::lock_guard<std::mutex> lock(joint_pos_mutex_);
                    joint_pos_[i] = low_state.motor_state()[i].q();
                    joint_vel_[i] = low_state.motor_state()[i].dq();
                }
                
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

        void ApplyHighLevelControl(const unitree_high_level_ctrl_msg& msg) override {
            if (exec_fsm_state_ != ExecFSMState::HIGH_LEVEL_CTRL) {
                RCLCPP_DEBUG_STREAM(this->get_logger(), "Ignoring high level control commands!");
                return;
            }

            sport_client_.Move(*msg.v_x, *msg.v_y, *msg.w_z);
        }

        bool CheckDampingToHomeTransition() {
            // TODO: check if system can safely transition to home position
            return false;
        }

        void TransitionToHome() override{
            RCLCPP_INFO_STREAM(this->get_logger(), "EXECUTION FSM TRANSTIONING TO HOME!");
            // First, save current position
            float _startPos[12];
            {
                std::lock_guard<std::mutex> lock(joint_pos_mutex_);
                std::copy(std::begin(joint_pos_), std::end(joint_pos_), _startPos);
            }
            

            // TODO: User defined home position
            float _homePos[12] = {0.0, 1.36, -2.65, 0.0, 1.36, -2.65,
                                     -0.2, 1.36, -2.65, 0.2, 1.36, -2.65};
            
            // Then loop
            float t = 0;
            float duration = 2;
            float proportion = 0;
            const int loop_rate_hz = 100;
            const std::chrono::milliseconds loop_period(1000 / loop_rate_hz); // 100Hz -> 10ms

            auto start_time = std::chrono::steady_clock::now();
            while (t < duration) {
                // Grab start of the loop time
                auto loop_start = std::chrono::steady_clock::now();

                // Check proprotion of completed movement
                proportion = std::min(t / duration, 1.0f);

                // Send command to the motors
                LowCmd_ dds_low_command;
                for (size_t j = 0; j < num_motors_; j++) {
                    dds_low_command.motor_cmd().at(j).mode() = 1;  // 1:Enable, 0:Disable
                    dds_low_command.motor_cmd().at(j).tau() = 0.;
                    dds_low_command.motor_cmd().at(j).q() = (1 - proportion) * _startPos[j] + proportion * _homePos[j];
                    dds_low_command.motor_cmd().at(j).dq() = 0.;
                    dds_low_command.motor_cmd().at(j).kp() = kp_[j];
                    dds_low_command.motor_cmd().at(j).kd() = kd_[j];
                }

                dds_low_command.crc() = Crc32Core((uint32_t *)&dds_low_command, (sizeof(dds_low_command) >> 2) - 1);
                lowcmd_publisher_->Write(dds_low_command);

                // Handle timing
                auto now = std::chrono::steady_clock::now();
                t = std::chrono::duration<float>(now - start_time).count();

                // Sleep to maintain 100Hz loop rate
                std::this_thread::sleep_until(loop_start + loop_period);
            }
            RCLCPP_INFO_STREAM(this->get_logger(), "EXECUTION FSM TRANSTION TO HOME COMPLETE!");
        }

        void TransitionToDamping() override {
            RCLCPP_INFO_STREAM(this->get_logger(), "EXECUTION FSM TRANSTION TO DAMPING!");
            // ---------- Create Packet ---------- //
            LowCmd_ dds_low_command;

            for (size_t j = 0; j < num_motors_; j++) {
                dds_low_command.motor_cmd().at(j).mode() = 1;  // 1:Enable, 0:Disable
                dds_low_command.motor_cmd().at(j).tau() = 0.;
                dds_low_command.motor_cmd().at(j).q() = 0.;
                dds_low_command.motor_cmd().at(j).dq() = 0.;
                dds_low_command.motor_cmd().at(j).kp() = 0.;
                dds_low_command.motor_cmd().at(j).kd() = kd_[j];
            }

            dds_low_command.crc() = Crc32Core((uint32_t *)&dds_low_command, (sizeof(dds_low_command) >> 2) - 1);
            lowcmd_publisher_->Write(dds_low_command);
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

        float joint_pos_[GO2_MOTOR_NUM];  // Local copy of joint positions
        float joint_vel_[GO2_MOTOR_NUM];  // Local copy of joint velocities
        std::mutex joint_pos_mutex_;      // mutex for copying joint positions and velocities
        go2::SportClient sport_client_;
        
    };
}   // namespace obelisk