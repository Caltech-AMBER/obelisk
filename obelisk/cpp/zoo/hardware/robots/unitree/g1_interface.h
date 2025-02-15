#include "unitree_interface.h"

// Locomotion Client
#include <unitree/robot/g1/loco/g1_loco_api.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>

// IDL
#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/hg/IMUState_.hpp>


namespace obelisk {

    using namespace unitree_hg::msg::dds_;
    
    class G1Interface : public ObeliskUnitreeInterface {
    public:
        G1Interface(const std::string& node_name)
            : ObeliskUnitreeInterface(node_name), use_hands_(false), fixed_waist_(true), mode_pr_(AnkleMode::PR), mode_machine_(0) {

            this->declare_parameter<bool>("use_hands", false);
            use_hands_ = this->get_parameter("use_hands").as_bool();

            this->declare_parameter<bool>("fixed_waist", true);
            fixed_waist_ = this->get_parameter("fixed_waist").as_bool();

            // Set G1 specific values
            num_motors_ = G1_27DOF;
            if (use_hands_) {
                num_motors_ += G1_HAND_MOTOR_NUM;
            } 

            if (!fixed_waist_) {
                num_motors_ += G1_EXTRA_WAIST;
            }

            for (int i = 0; i < G1_27DOF; i++) {
                joint_names_.push_back(G1_27DOF_JOINT_NAMES[i]);
            }
            if (!fixed_waist_) {
                for (int i = 0; i < G1_EXTRA_WAIST; i++) {
                    joint_names_.push_back(G1_EXTRA_WAIST_JOINT_NAMES[i]);
                }
            }

            // Setup local variables
            // TODO: For now we are not supporting the hands here
            joint_pos_.resize(G1_27DOF + G1_EXTRA_WAIST);
            joint_vel_.resize(G1_27DOF + G1_EXTRA_WAIST);
            start_user_pose_.resize(G1_27DOF + G1_EXTRA_WAIST);
            user_pose_.resize(G1_27DOF + G1_EXTRA_WAIST);
            
            CMD_TOPIC_ = "rt/lowcmd";
            IMU_TORSO_ = "rt/secondary_imu";
            STATE_TOPIC_ = "rt/lowstate";
            ODOM_TOPIC_ = "rt/odommodestate";

            // TODO: Consider exposing AB mode

            // TODO: Deal with hands

            RCLCPP_INFO_STREAM(this->get_logger(), "G1 configured as: \n" << "\tHands: " << use_hands_ << "\n\tFixed waist: " << fixed_waist_);

            VerifyParameters();
            CreateUnitreePublishers();

            loco_client_.SetTimeout(10.0f);
            loco_client_.Init();
        }
    protected:
        void CreateUnitreePublishers() override {
            // create publisher
            lowcmd_publisher_.reset(new ChannelPublisher<LowCmd_>(CMD_TOPIC_));
            lowcmd_publisher_->InitChannel();

            RCLCPP_INFO_STREAM(this->get_logger(), "G1 command publishers created!");
        }

        void CreateUnitreeSubscribers() override {
            // create subscriber
            // Need to create after the publishers have been activated
            lowstate_subscriber_.reset(new ChannelSubscriber<LowState_>(STATE_TOPIC_));
            lowstate_subscriber_->InitChannel(std::bind(&G1Interface::LowStateHandler, this, std::placeholders::_1), 10);

            // odom_subscriber_.reset(new ChannelSubscriber<IMUState_>(HG_ODOM_TOPIC));
            // odom_subscriber_->InitChannel(std::bind(&ObeliskUnitreeInterface::OdomHandler, this, std::placeholders::_1), 10);
        }

        void ApplyControl(const unitree_control_msg& msg) override {
            // Only execute of in Low Level Control or Home modes
            if (exec_fsm_state_ != ExecFSMState::LOW_LEVEL_CTRL && exec_fsm_state_ != ExecFSMState::USER_POSE) {
                return;
            }
            RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Applying control to Unitree robot!");

            // ---------- Create Packet ---------- //
            LowCmd_ dds_low_command;
            dds_low_command.mode_pr() = static_cast<uint8_t>(mode_pr_);
            dds_low_command.mode_machine() = mode_machine_;

            // ------------------------ Execution FSM: Low Level Control ------------------------ //
            if (exec_fsm_state_ == ExecFSMState::LOW_LEVEL_CTRL) {
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
                        }
                    }
                }
                for (size_t j = 0; j < num_motors_; j++) {
                    if (!joint_used[j]) {
                        throw std::runtime_error("[UnitreeRobotInterface] Control message is missing robot joints!");
                    }
                }

                // Setup gains
                bool use_default_gains = true;
                if (msg.joint_names.size() == msg.kp.size()) {
                    if (msg.kp.size() != num_motors_) {
                        throw std::runtime_error("[UnitreeRobotInterface] Control message gains are not of the right size!");
                    }
                    use_default_gains = false;
                }

                for (size_t j = 0; j < G1_27DOF; j++) {     // Only go through the non-hand motors
                    int i = G1_JOINT_MAPPINGS.at(msg.joint_names[j]);
                    dds_low_command.motor_cmd().at(i).mode() = 1;  // 1:Enable, 0:Disable
                    dds_low_command.motor_cmd().at(i).tau() = msg.feed_forward[j];
                    dds_low_command.motor_cmd().at(i).q() = msg.pos_target[j];
                    dds_low_command.motor_cmd().at(i).dq() = msg.vel_target[j];
                    dds_low_command.motor_cmd().at(i).kp() = use_default_gains ? kp_[i] : msg.kp[j];
                    dds_low_command.motor_cmd().at(i).kd() = use_default_gains ? kd_[i] : msg.kd[j];
                }
                if (fixed_waist_) {
                    for (size_t j = 0; j < G1_EXTRA_WAIST; j++) {
                        int i = G1_JOINT_MAPPINGS.at(G1_EXTRA_WAIST_JOINT_NAMES[j]);
                        dds_low_command.motor_cmd().at(i).mode() = 0;  // 1:Enable, 0:Disable
                        dds_low_command.motor_cmd().at(i).tau() = 0;
                        dds_low_command.motor_cmd().at(i).q() = 0;
                        dds_low_command.motor_cmd().at(i).dq() = 0;
                        dds_low_command.motor_cmd().at(i).kp() = 0;
                        dds_low_command.motor_cmd().at(i).kd() = 0;
                    }
                }
            // ------------------------ Execution FSM: Home ------------------------ //
            } else if (exec_fsm_state_ == ExecFSMState::USER_POSE) {
                // Compute time that robot has been in HOME
                float t = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - user_pose_transition_start_time_).count();
                // Compute proportion of time relative to transition duration
                float proportion = std::min(t / user_pose_transition_duration_, 1.0f);
                // Write message
                for (size_t i = 0; i < G1_27DOF + G1_EXTRA_WAIST; i++) {    // Sending the extra waist commands while in fixed waist should have no effect
                    dds_low_command.motor_cmd().at(i).mode() = 1;  // 1:Enable, 0:Disable
                    dds_low_command.motor_cmd().at(i).tau() = 0.;
                    dds_low_command.motor_cmd().at(i).q() = (1 - proportion) * start_user_pose_[i] + proportion * user_pose_[i];
                    dds_low_command.motor_cmd().at(i).dq() = 0.;
                    dds_low_command.motor_cmd().at(i).kp() = kp_[i];
                    dds_low_command.motor_cmd().at(i).kd() = kd_[i];
                }
            } else {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Execution FSM state not recognized in ApplyControl!");
            }

            // Write command to the robot
            dds_low_command.crc() = Crc32Core((uint32_t *)&dds_low_command, (sizeof(dds_low_command) >> 2) - 1);
            lowcmd_publisher_->Write(dds_low_command);
            
            // TODO: Deal with the hands
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
                // if (low_state.motor_state()[i].motorstate() && i <= RightAnkleRoll) // TODO: What is this?
                // std::cout << "[ERROR] motor " << i << " with code " << low_state.motor_state()[i].motorstate() << "\n";
            }

            this->GetPublisher<obelisk_sensor_msgs::msg::ObkJointEncoders>(pub_joint_state_key_)->publish(joint_state);

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

            this->GetPublisher<obelisk_sensor_msgs::msg::ObkImu>(pub_imu_state_key_)->publish(imu_state);

            // update mode machine
            if (mode_machine_ != low_state.mode_machine()) {
                if (mode_machine_ == 0) std::cout << "G1 type: " << unsigned(low_state.mode_machine()) << std::endl;
                mode_machine_ = low_state.mode_machine();
            }
        }

        void ApplyHighLevelControl(const unitree_high_level_ctrl_msg& msg) override {
            if (exec_fsm_state_ != ExecFSMState::HIGH_LEVEL_CTRL) {
                return;
            }

            loco_client_.SetVelocity(msg.v_x, msg.v_y, msg.w_z);
        }

        bool CheckDampingToUnitreeHomeTransition() {
            // TODO: check if system can safely transition to home position
            return false;
        }

        void TransitionToUserPose() override{
            RCLCPP_INFO_STREAM(this->get_logger(), "EXECUTION FSM TRANSTION TO HOME!");
            // First, save current position
            {
                std::lock_guard<std::mutex> lock(joint_pos_mutex_);
                joint_pos_ = start_user_pose_;
            }
            user_pose_transition_start_time_ = std::chrono::steady_clock::now();  // Save start time
        }

        void TransitionToUnitreeHome() override{
            loco_client_.BalanceStand();
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

        bool use_hands_;
        bool fixed_waist_;

        std::string IMU_TORSO_;
        std::string ODOM_TOPIC_;

        unitree::robot::ChannelPublisherPtr<LowCmd_> lowcmd_publisher_;
        ChannelSubscriberPtr<LowState_> lowstate_subscriber_;
        // ChannelSubscriberPtr<IMUState_> odom_subscriber_;


        enum class AnkleMode {
            PR = 0,  // Series Control for Ptich/Roll Joints
            AB = 1   // Parallel Control for A/B Joints
        };

        AnkleMode mode_pr_;
        uint8_t mode_machine_;

        // Locomotion Client
        g1::LocoClient loco_client_;

    private:
        // ---------- Robot Specific ---------- //
        // G1
        static constexpr int G1_27DOF = 27;
        static constexpr int G1_EXTRA_WAIST = 2;
        static constexpr int G1_HAND_MOTOR_NUM = 14;

        float user_pose_transition_duration_;                     // Duration of the transition to home position

        std::vector<double> joint_pos_; // Local copy of joint positions
        std::vector<double> joint_vel_; // Local copy of joint positions
        std::vector<double> start_user_pose_; // For transitioning to home position
        std::vector<double> user_pose_; // home position
        // float joint_pos_[G1_MOTOR_NUM + G1_HAND_MOTOR_NUM];  
        // float joint_vel_[G1_MOTOR_NUM + G1_HAND_MOTOR_NUM];  // Local copy of joint velocities
        // float start_user_pose_[G1_MOTOR_NUM + G1_HAND_MOTOR_NUM];     // For transitioning to home position
        // float user_pose_[G1_MOTOR_NUM + G1_HAND_MOTOR_NUM];           // home position

        std::mutex joint_pos_mutex_;              // mutex for copying joint positions and velocities
        std::chrono::steady_clock::time_point user_pose_transition_start_time_;             // Timing variable for transition to stand
        std::vector<double> default_user_pose_ = {
            0.3, 0, 0, 0.52, -0.23, 0,   // Left Leg
            -0.3, 0, 0, 0.52, -0.23, 0,  // Right Leg
            0, 0, 0.03,                  // Waist
            0, 0, 0, 0.2, 0, 0, 0,       // Left arm
            0, 0, 0, 0.2, 0, 0, 0,       // Right arm
            0, 0, 0, 0, 0, 0, 0,         // Left hand
            0, 0, 0, 0, 0, 0, 0          // Right hand
        };   // Default home position
            
        // const std::array<std::string, G1_27DOF + G1_EXTRA_WAIST + G1_HAND_MOTOR_NUM> G1_FULL_JOINT_NAMES = {
        //     "left_hip_pitch_joint",
        //     "left_hip_roll_joint",
        //     "left_hip_yaw_joint",
        //     "left_knee_joint",
        //     "left_ankle_pitch_joint",
        //     "left_ankle_roll_joint",
        //     "right_hip_pitch_joint",
        //     "right_hip_roll_joint",
        //     "right_hip_yaw_joint",
        //     "right_knee_joint",
        //     "right_ankle_pitch_joint",
        //     "right_ankle_roll_joint",
        //     "waist_yaw_joint",
        //     "waist_roll_joint",
        //     "waist_pitch_joint",
        //     "left_shoulder_pitch_joint",
        //     "left_shoulder_roll_joint",
        //     "left_shoulder_yaw_joint",
        //     "left_elbow_joint",
        //     "left_wrist_roll_joint",
        //     "left_wrist_pitch_joint",
        //     "left_wrist_yaw_joint",
        //     "right_shoulder_pitch_joint",
        //     "right_shoulder_roll_joint",
        //     "right_shoulder_yaw_joint",
        //     "right_elbow_joint",
        //     "right_wrist_roll_joint",
        //     "right_wrist_pitch_joint",
        //     "right_wrist_yaw_joint",
        //     "left_hand_thumb_0_joint",      // ----- Hand Start ----- //
        //     "left_hand_thumb_1_joint",
        //     "left_hand_thumb_2_joint",
        //     "left_hand_middle_0_joint",
        //     "left_hand_middle_1_joint",
        //     "left_hand_index_0_joint",
        //     "left_hand_index_1_joint",
        //     "right_hand_thumb_0_joint",      
        //     "right_hand_thumb_1_joint",
        //     "right_hand_thumb_2_joint",
        //     "right_hand_middle_0_joint",
        //     "right_hand_middle_1_joint",
        //     "right_hand_index_0_joint",
        //     "right_hand_index_1_joint"
        // };

        const std::array<std::string, G1_27DOF> G1_27DOF_JOINT_NAMES = {
            "left_hip_pitch_joint",
            "left_hip_roll_joint",
            "left_hip_yaw_joint",
            "left_knee_joint",
            "left_ankle_pitch_joint",
            "left_ankle_roll_joint",
            "right_hip_pitch_joint",
            "right_hip_roll_joint",
            "right_hip_yaw_joint",
            "right_knee_joint",
            "right_ankle_pitch_joint",
            "right_ankle_roll_joint",
            "waist_yaw_joint",
            "left_shoulder_pitch_joint",
            "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint",
            "left_elbow_joint",
            "left_wrist_roll_joint",
            "left_wrist_pitch_joint",
            "left_wrist_yaw_joint",
            "right_shoulder_pitch_joint",
            "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint",
            "right_elbow_joint",
            "right_wrist_roll_joint",
            "right_wrist_pitch_joint",
            "right_wrist_yaw_joint",
        };

        const std::array<std::string, G1_EXTRA_WAIST> G1_EXTRA_WAIST_JOINT_NAMES = {
            "waist_roll_joint",
            "waist_pitch_joint"
        };

        const std::array<std::string, G1_HAND_MOTOR_NUM> G1_HAND_JOINT_NAMES = {
            "left_hand_thumb_0_joint",
            "left_hand_thumb_1_joint",
            "left_hand_thumb_2_joint",
            "left_hand_middle_0_joint",
            "left_hand_middle_1_joint",
            "left_hand_index_0_joint",
            "left_hand_index_1_joint",
            "right_hand_thumb_0_joint",      
            "right_hand_thumb_1_joint",
            "right_hand_thumb_2_joint",
            "right_hand_middle_0_joint",
            "right_hand_middle_1_joint",
            "right_hand_index_0_joint",
            "right_hand_index_1_joint"
        };

        // enum G1JointIndex {
        //     LeftHipPitch = 0,
        //     LeftHipRoll = 1,
        //     LeftHipYaw = 2,
        //     LeftKnee = 3,
        //     LeftAnklePitch = 4,
        //     LeftAnkleB = 4,
        //     LeftAnkleRoll = 5,
        //     LeftAnkleA = 5,
        //     RightHipPitch = 6,
        //     RightHipRoll = 7,
        //     RightHipYaw = 8,
        //     RightKnee = 9,
        //     RightAnklePitch = 10,
        //     RightAnkleB = 10,
        //     RightAnkleRoll = 11,
        //     RightAnkleA = 11,
        //     WaistYaw = 12,
        //     WaistRoll = 13,        // NOTE INVALID for g1 23dof/29dof with waist locked
        //     WaistA = 13,           // NOTE INVALID for g1 23dof/29dof with waist locked
        //     WaistPitch = 14,       // NOTE INVALID for g1 23dof/29dof with waist locked
        //     WaistB = 14,           // NOTE INVALID for g1 23dof/29dof with waist locked
        //     LeftShoulderPitch = 15,
        //     LeftShoulderRoll = 16,
        //     LeftShoulderYaw = 17,
        //     LeftElbow = 18,
        //     LeftWristRoll = 19,
        //     LeftWristPitch = 20,   // NOTE INVALID for g1 23dof
        //     LeftWristYaw = 21,     // NOTE INVALID for g1 23dof
        //     RightShoulderPitch = 22,
        //     RightShoulderRoll = 23,
        //     RightShoulderYaw = 24,
        //     RightElbow = 25,
        //     RightWristRoll = 26,
        //     RightWristPitch = 27,  // NOTE INVALID for g1 23dof
        //     RightWristYaw = 28     // NOTE INVALID for g1 23dof
        // };

        const std::map<std::string, int> G1_JOINT_MAPPINGS = {
            {"left_hip_pitch_joint", 0},
            {"left_hip_roll_joint", 1},
            {"left_hip_yaw_joint", 2},
            {"left_knee_joint", 3},
            {"left_ankle_pitch_joint", 4},
            {"left_ankle_roll_joint", 5},
            {"right_hip_pitch_joint", 6},
            {"right_hip_roll_joint", 7},
            {"right_hip_yaw_joint", 8},
            {"right_knee_joint", 9},
            {"right_ankle_pitch_joint", 10},
            {"right_ankle_roll_joint", 11},
            {"waist_yaw_joint", 12},
            {"waist_roll_joint", 13},
            {"waist_pitch_joint", 14},
            {"left_shoulder_pitch_joint", 15},
            {"left_shoulder_roll_joint", 16},
            {"left_shoulder_yaw_joint", 17},
            {"left_elbow_joint", 18},
            {"left_wrist_roll_joint", 19},
            {"left_wrist_pitch_joint", 20},
            {"left_wrist_yaw_joint", 21},
            {"right_shoulder_pitch_joint", 22},
            {"right_shoulder_roll_joint", 23},
            {"right_shoulder_yaw_joint", 24},
            {"right_elbow_joint", 25},
            {"right_wrist_roll_joint", 26},
            {"right_wrist_pitch_joint", 27},
            {"right_wrist_yaw_joint", 28}
        };
    };
}   // namespace obelisk