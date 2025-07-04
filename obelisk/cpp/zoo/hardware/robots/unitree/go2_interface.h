#include "unitree_interface.h"

#include "obelisk_sensor_msgs/msg/obk_force_sensor.hpp"

// Sport Client
#include <unitree/robot/go2/sport/sport_client.hpp>

// IDL
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/LowState_.hpp>

namespace obelisk {

    using namespace unitree_go::msg::dds_;

    class Go2Interface : public ObeliskUnitreeInterface {
      public:
        Go2Interface(const std::string& node_name) : ObeliskUnitreeInterface(node_name) {

            // Additional Publishers
            this->RegisterObkPublisher<obelisk_sensor_msgs::msg::ObkForceSensor>("pub_force_sensor_setting",
                                                                                 "force_sensor_key");

            // Expose duration of transition to home position as ros parameter
            this->declare_parameter<float>("user_pose_transition_duration", 1.);
            user_pose_transition_duration_ = this->get_parameter("user_pose_transition_duration").as_double();

            // Expose high level velocity deadzone as a ros parameter
            this->declare_parameter<float>("velocity_deadzone", 0.01);
            vel_deadzone_ = this->get_parameter("velocity_deadzone").as_double();

            // Expose home position as a ros parameter
            this->declare_parameter<std::vector<double>>("home_position", default_user_pose_);
            auto user_pose_vector = this->get_parameter("home_position").as_double_array(); // Get as vector
            std::copy(user_pose_vector.begin(), user_pose_vector.end(), user_pose_);

            // Handle joint names
            num_motors_ = GO2_MOTOR_NUM;
            for (size_t i = 0; i < num_motors_; i++) {
                joint_names_.push_back(GO2_JOINT_NAMES[i]);
            }

            // Define topic names
            CMD_TOPIC_   = "rt/lowcmd";
            STATE_TOPIC_ = "rt/lowstate";
            // TODO: Look for ODOM topic

            // Verify params and create unitree publishers
            VerifyParameters();
            CreateUnitreePublishers();

            // Initialize sports client
            sport_client_.SetTimeout(10.0f);
            sport_client_.Init();
        }

      protected:
        void CreateUnitreePublishers() override {
            // create low level command publisher
            lowcmd_publisher_.reset(new ChannelPublisher<LowCmd_>(CMD_TOPIC_));
            lowcmd_publisher_->InitChannel();

            RCLCPP_INFO_STREAM(this->get_logger(), "Go2 command publishers created!");
        }

        void CreateUnitreeSubscribers() override {
            // Dreate unitree subscriber
            // Need to create after the publishers have been activated
            lowstate_subscriber_.reset(new ChannelSubscriber<LowState_>(STATE_TOPIC_));
            lowstate_subscriber_->InitChannel(std::bind(&Go2Interface::LowStateHandler, this, std::placeholders::_1),
                                              1);

            // Odom state?
        }

        void ApplyControl(const unitree_control_msg& msg) override {
            // Only execute of in Low Level Control or Home modes
            if (exec_fsm_state_ != ExecFSMState::LOW_LEVEL_CTRL && exec_fsm_state_ != ExecFSMState::USER_POSE) {
                return;
            }
            RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Applying control to Unitree robot!");

            // Create packet
            LowCmd_ dds_low_command;

            // ------------------------ Execution FSM: Low Level Control ------------------------ //
            if (exec_fsm_state_ == ExecFSMState::LOW_LEVEL_CTRL) {
                // ---------- Verify message ---------- //
                if (msg.joint_names.size() != msg.pos_target.size() ||
                    msg.joint_names.size() != msg.vel_target.size() ||
                    msg.joint_names.size() != msg.feed_forward.size()) {
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
                std::vector<int> joint_mapping(num_motors_, -1); // Maps the j'th input joint to the i'th output joint
                if (msg.joint_names.size() != joint_names_.size()) {
                    throw std::runtime_error("[UnitreeRobotInterface] Control message joint name size does not match "
                                             "robot joint name size!");
                }

                for (size_t j = 0; j < num_motors_; j++) {
                    for (size_t i = 0; i < num_motors_; i++) {
                        if (msg.joint_names[j] == joint_names_[i]) {
                            if (joint_used[i]) {
                                throw std::runtime_error(
                                    "[UnitreeRobotInterface] Control message uses the same joint name twice!");
                            }

                            joint_used[i]    = true;
                            joint_mapping[j] = i;
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
                        throw std::runtime_error(
                            "[UnitreeRobotInterface] Control message gains are not of the right size!");
                    }
                    use_default_gains = false;
                }

                // Write message
                for (size_t j = 0; j < num_motors_; j++) {        // Only go through the non-hand motors
                    int i                                    = joint_mapping[j];
                    dds_low_command.motor_cmd().at(i).mode() = 1; // 1:Enable, 0:Disable
                    dds_low_command.motor_cmd().at(i).tau()  = msg.feed_forward[j];
                    dds_low_command.motor_cmd().at(i).q()    = msg.pos_target[j];
                    dds_low_command.motor_cmd().at(i).dq()   = msg.vel_target[j];
                    dds_low_command.motor_cmd().at(i).kp()   = use_default_gains ? kp_[i] : msg.kp[j];
                    dds_low_command.motor_cmd().at(i).kd()   = use_default_gains ? kd_[i] : msg.kd[j];
                }

                // ------------------------ Execution FSM: user_pose ------------------------ //
            } else if (exec_fsm_state_ == ExecFSMState::USER_POSE) {
                // Compute time that robot has been in HOME
                float t =
                    std::chrono::duration<double>(std::chrono::steady_clock::now() - user_pose_transition_start_time_)
                        .count();
                // Compute proportion of time relative to transition duration
                float proportion = std::min(t / user_pose_transition_duration_, 1.0f);
                // Write message
                for (size_t i = 0; i < num_motors_; i++) {
                    dds_low_command.motor_cmd().at(i).mode() = 1; // 1:Enable, 0:Disable
                    dds_low_command.motor_cmd().at(i).tau()  = 0.;
                    dds_low_command.motor_cmd().at(i).q() =
                        (1 - proportion) * start_user_pose_[i] + proportion * user_pose_[i];
                    dds_low_command.motor_cmd().at(i).dq() = 0.;
                    dds_low_command.motor_cmd().at(i).kp() = kp_[i];
                    dds_low_command.motor_cmd().at(i).kd() = kd_[i];
                }
            } else {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Execution FSM state not recognized in ApplyControl!");
            }

            // Write command to the robot
            dds_low_command.crc() = Crc32Core((uint32_t*)&dds_low_command, (sizeof(dds_low_command) >> 2) - 1);
            lowcmd_publisher_->Write(dds_low_command);
        }

        void LowStateHandler(const void* message) {
            LowState_ low_state = *(const LowState_*)message;
            if (low_state.crc() != Crc32Core((uint32_t*)&low_state, (sizeof(LowState_) >> 2) - 1)) {
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
                joint_state.joint_pos.at(i)   = low_state.motor_state()[i].q();
                joint_state.joint_vel.at(i)   = low_state.motor_state()[i].dq();
                {
                    std::lock_guard<std::mutex> lock(joint_pos_mutex_);
                    joint_pos_[i] = low_state.motor_state()[i].q();
                    joint_vel_[i] = low_state.motor_state()[i].dq();
                }
                if (low_state.motor_state()[i].lost() > 0) {
                    static std::array<unsigned int, GO2_MOTOR_NUM> last_loss = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
                    if (low_state.motor_state()[i].lost() != last_loss[i]) {
                        RCLCPP_ERROR_STREAM(this->get_logger(),
                                            "Motor " << joint_names_[i]
                                                     << " lost: " << low_state.motor_state()[i].lost() << " packets!");
                    }
                    last_loss[i] = low_state.motor_state()[i].lost();
                }
            }

            this->GetPublisher<obelisk_sensor_msgs::msg::ObkJointEncoders>(pub_joint_state_key_)->publish(joint_state);

            // IMU
            obelisk_sensor_msgs::msg::ObkImu imu_state;
            imu_state.header.stamp       = this->now();
            imu_state.angular_velocity.x = low_state.imu_state().gyroscope()[0];
            imu_state.angular_velocity.y = low_state.imu_state().gyroscope()[1];
            imu_state.angular_velocity.z = low_state.imu_state().gyroscope()[2];

            imu_state.orientation.w = low_state.imu_state().quaternion()[0];
            imu_state.orientation.x = low_state.imu_state().quaternion()[1];
            imu_state.orientation.y = low_state.imu_state().quaternion()[2];
            imu_state.orientation.z = low_state.imu_state().quaternion()[3];

            imu_state.linear_acceleration.x = low_state.imu_state().accelerometer()[0];
            imu_state.linear_acceleration.y = low_state.imu_state().accelerometer()[1];
            imu_state.linear_acceleration.z = low_state.imu_state().accelerometer()[2];

            this->GetPublisher<obelisk_sensor_msgs::msg::ObkImu>(pub_imu_state_key_)->publish(imu_state);

            // Foot forces
            obelisk_sensor_msgs::msg::ObkForceSensor force_sensor;
            force_sensor.header.stamp = this->now();
            force_sensor.forces.resize(4);
            force_sensor.frames = {"FR_foot", "FL_foot", "RR_foot", "RL_foot"};
            for (int i = 0; i < 4; i++) {
                force_sensor.forces[i] = low_state.foot_force()[i];
            }
            this->GetPublisher<obelisk_sensor_msgs::msg::ObkForceSensor>("force_sensor_key")->publish(force_sensor);
        }

        void ApplyHighLevelControl(const unitree_high_level_ctrl_msg& msg) override {
            // Ignore if not in high-level mode
            if (exec_fsm_state_ != ExecFSMState::HIGH_LEVEL_CTRL) {
                return;
            }

            // Include logic to stand when velocity is within deadzone
            static bool standing = false;
            if (msg.v_x * msg.v_x + msg.v_y * msg.v_y + msg.w_z * msg.w_z > vel_deadzone_) {
                if (standing) {
                    sport_client_.SwitchGait(1); // Set gait to trot
                    standing = false;
                }
                sport_client_.Move(msg.v_x, msg.v_y, msg.w_z); // Command velocity
            } else {
                if (!standing) {
                    sport_client_.SwitchGait(0); // Set gait to stand
                    standing = true;
                }
            }
        }

        bool CheckDampingToUnitreeHomeTransition() {
            // Define prone joint angles
            const float prone[12] = {0., 1.26, -2.8, 0.0, 1.26, -2.8, -0.35, 1.3, -2.82, 0.35, 1.3, -2.82};
            // Check for low velocities and joint angles near prone
            for (size_t i = 0; i < num_motors_; i++) {
                if (abs(joint_vel_[i]) > 0.1 or abs(joint_pos_[i] - prone[i]) > 0.1) {
                    return false;
                }
            }
            return true;
        }

        void TransitionToUserPose() override {
            RCLCPP_INFO_STREAM(this->get_logger(), "EXECUTION FSM TRANSTION TO user_pose!");
            // First, save current position
            {
                std::lock_guard<std::mutex> lock(joint_pos_mutex_);
                std::copy(std::begin(joint_pos_), std::end(joint_pos_), start_user_pose_);
            }
            user_pose_transition_start_time_ = std::chrono::steady_clock::now(); // Save start time
        }

        void TransitionToUnitreeHome() override { sport_client_.BalanceStand(); }

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
        static constexpr int GO2_MOTOR_NUM = 12; // Number of motors

        float joint_pos_[GO2_MOTOR_NUM];         // Local copy of joint positions
        float joint_vel_[GO2_MOTOR_NUM];         // Local copy of joint velocities
        float start_user_pose_[GO2_MOTOR_NUM];   // For transitioning to home position
        float user_pose_transition_duration_;    // Duration of the transition to home position
        float vel_deadzone_;                     // Deadzone for high level velocity commands
        std::vector<double> default_user_pose_ = {0.0, 0.9, -1.8, 0.0, 0.9, -1.8,
                                                  0.0, 0.9, -1.8, 0.0, 0.9, -1.8}; // Default home position
        float user_pose_[GO2_MOTOR_NUM];                                           // home position
        std::mutex joint_pos_mutex_;          // mutex for copying joint positions and velocities
        go2::SportClient sport_client_;       // Sports client for high level control
        std::chrono::steady_clock::time_point
            user_pose_transition_start_time_; // Timing variable for transition to stand
        const std::array<std::string, GO2_MOTOR_NUM> GO2_JOINT_NAMES = {
            // Names of joints, in order for hardware
            "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
            "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint", "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};
    };
} // namespace obelisk
