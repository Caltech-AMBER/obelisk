#include "obelisk_robot.h"
#include "obelisk_ros_utils.h"
#include "obelisk_sensor_msgs/msg/obk_joint_encoders.h"
#include "obelisk_control_msgs/msg/pd_feed_forward.h"

// DDS
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

// IDL
#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

namespace obelisk {
    using unitree_control_msg = obelisk_control_msgs::msg::PDFeedForward;
    using unitree_estimator_msg  = obelisk_sensor_msgs::msg::ObkJointEncoders;

    // TODO: Verify these are the same of the G1 and the Go2
    static const std::string HG_CMD_TOPIC = "rt/lowcmd";
    static const std::string HG_IMU_TORSO = "rt/secondary_imu";
    static const std::string HG_STATE_TOPIC = "rt/lowstate";

    using namespace unitree::common;
    using namespace unitree::robot;
    using namespace unitree_hg::msg::dds_;

    /**
     * @brief Obelisk robot interface for the Leap hand hardware
     */
    class ObeliskUnitreeInterface : public ObeliskRobot<unitree_control_msg> {

    public:
        ObeliskUnitreeInterface(const std::string& node_name)
            : ObeliskRobot<unitree_control_msg>(node_name), mode_pr_(AnkleMode::PR), mode_machine_(0) {

            // Get network interface name as a parameter
            this->declare_parameter<std::string>("network_interface_name", "");
            network_interface_name_ = this->get_parameter("network_interface_name").as_string();

            // ---------- Default PD gains ---------- //
            // TODO: Make this a vector
            this->declare_parameter<std::vector<double>>("default_kp");
            this->declare_parameter<std::vector<double>>("default_kd");
            kp_ = this->get_parameter("default_kp").as_double_array();
            kd_ = this->get_parameter("default_kd").as_double_array();

            // TODO: User defined safety maybe
            
            // TODO: Add option for waist locked

            // TODO: Consider exposing AB mode

            // TODO: Deal with hands

            // ---------- Get the robot name (G1, Go2) ---------- //
            this->declare_parameter<std::string>("robot_name", "");
            name_ = this->get_parameter("robot_name").as_string();
            if (name_ != "G1" && name_ != "Go2") {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Provided name: " << name_);
                throw std::runtime_error("[UnitreeRobotInterface] Unitree robot name is invalid!");
            }

            // TODO: Remove!
            if (name_ != "G1") {
                throw std::runtime_error("[UnitreeRobotInterface] Robot not implemented yet!");
            }

            if (name_ == "G1") {
                num_motors_ = G1_MOTOR_NUM;
                for (int i = 0; i < G1_MOTOR_NUM; i++) {
                    joint_names_.push_back(G1_JOINT_NAMES[i]);
                } 
            } else if (name_ == "Go2") {
                throw std::runtime_error("[UnitreeRobotInterface] Not implemented!");
            }

            // Verify default gains
            if (kp_.empty()) {
                for (size_t i = 0; i < num_motors_; i++) {
                    kp_.push_back(0);
                }
            }

            if (kd_.empty()) {
                for (size_t i = 0; i < num_motors_; i++) {
                    kd_.push_back(0);
                }
            }

            if (kp_.size() != num_motors_ || kd_.size() != num_motors_) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Expected size: " << num_motors_);
                RCLCPP_ERROR_STREAM(this->get_logger(), "default kp size: " << kp_.size());
                RCLCPP_ERROR_STREAM(this->get_logger(), "default kd size: " << kd_.size());
                throw std::runtime_error("[UnitreeRobotInterface] default gains have incorrect sizes!");
            }

            // Print Parameters
            RCLCPP_INFO_STREAM(this->get_logger(), "Network interface: " << network_interface_name_);
            // RCLCPP_INFO_STREAM(this->get_logger(), "default_kp: ");
            // for (size_t i = 0; i < kp_.size(); i++) {
            //     RCLCPP_INFO_STREAM(this->get_logger(), kp_[i] << ",");
            // }
            // RCLCPP_INFO_STREAM(this->get_logger(), "default_kd: ");
            // for (size_t i = 0; i < kp_.size(); i++) {
            //     RCLCPP_INFO_STREAM(this->get_logger(), kd_[i] << ",");
            // }

            ChannelFactory::Instance()->Init(0, network_interface_name_);

            // try to shutdown motion control-related service
            mode_switch_manager_ = std::make_shared<unitree::robot::b2::MotionSwitcherClient>();
            mode_switch_manager_->SetTimeout(5.0f);
            mode_switch_manager_->Init();
            std::string form, name;
            while (mode_switch_manager_->CheckMode(form, name), !name.empty()) {
                if (mode_switch_manager_->ReleaseMode())
                    std::cout << "Failed to switch to Release Mode\n";
                sleep(5);
            }

            // create publisher
            lowcmd_publisher_.reset(new ChannelPublisher<LowCmd_>(HG_CMD_TOPIC));
            lowcmd_publisher_->InitChannel();
            // create subscriber
            lowstate_subscriber_.reset(new ChannelSubscriber<LowState_>(HG_STATE_TOPIC));
            lowstate_subscriber_->InitChannel(std::bind(&ObeliskUnitreeInterface::LowStateHandler, this, std::placeholders::_1), 1);

            RCLCPP_INFO_STREAM(this->get_logger(), "Pubs and Subs created!");
        }
    protected:

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
            std::vector<bool> joint_used(num_motors_, false);
            std::vector<int> joint_mapping(num_motors_, -1);    // Maps the j'th input joint to the i'th output joint
            if (msg.joint_names.size() != joint_names_.size()) {
                throw std::runtime_error("[UnitreeRobotInterface] Control message joint name size does not match robot joint name size!");
            }

            for (size_t j = 0; j < num_motors_; j++) {
                for (size_t i = 0; i < num_motors_; i++) {
                    if (msg.joint_names[j] == joint_names_[i]) {
                        if (joint_used[i]) {
                            throw std::runtime_error("[UnitreeRobotInterface] Control message usess the same joint name twice!");
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
            dds_low_command.mode_pr() = static_cast<uint8_t>(mode_pr_);
            dds_low_command.mode_machine() = mode_machine_;

            for (size_t j = 0; j < num_motors_; j++) {
                int i = joint_mapping[j];
                dds_low_command.motor_cmd().at(i).mode() = 1;  // 1:Enable, 0:Disable
                dds_low_command.motor_cmd().at(i).tau() = msg.feed_forward[j];
                dds_low_command.motor_cmd().at(i).q() = msg.pos_target[j];
                dds_low_command.motor_cmd().at(i).dq() = msg.vel_target[j];
                dds_low_command.motor_cmd().at(i).kp() = use_default_gains ? kp_[i] : msg.kp[j];
                dds_low_command.motor_cmd().at(i).kd() = use_default_gains ? kd_[i] : msg.kd[j];
                
                // RCLCPP_INFO_STREAM(this->get_logger(), "[" << i << "]" << "data: " 
                //     << dds_low_command.motor_cmd().at(i).tau() << ", "
                //     << dds_low_command.motor_cmd().at(i).q() << ", "
                //     << dds_low_command.motor_cmd().at(i).dq() << ", "
                //     << dds_low_command.motor_cmd().at(i).kp() << ", "
                //     << dds_low_command.motor_cmd().at(i).kd() << ", ");
            }

            dds_low_command.crc() = Crc32Core((uint32_t *)&dds_low_command, (sizeof(dds_low_command) >> 2) - 1);
            lowcmd_publisher_->Write(dds_low_command);
        }

        void LowStateHandler(const void *message) {
            LowState_ low_state = *(const LowState_ *)message;
            if (low_state.crc() != Crc32Core((uint32_t *)&low_state, (sizeof(LowState_) >> 2) - 1)) {
                std::cout << "[ERROR] CRC Error" << std::endl;
                return;
            }
            // get motor state
            // MotorState ms_tmp;
            // for (int i = 0; i < G1_NUM_MOTOR; ++i) {
            //     ms_tmp.q.at(i) = low_state.motor_state()[i].q();
            //     ms_tmp.dq.at(i) = low_state.motor_state()[i].dq();
            //     if (low_state.motor_state()[i].motorstate() && i <= RightAnkleRoll)
            //     std::cout << "[ERROR] motor " << i << " with code " << low_state.motor_state()[i].motorstate() << "\n";
            // }
            // motor_state_buffer_.SetData(ms_tmp);
            // get imu state
            // ImuState imu_tmp;
            // imu_tmp.omega = low_state.imu_state().gyroscope();
            // imu_tmp.rpy = low_state.imu_state().rpy();
            // imu_state_buffer_.SetData(imu_tmp);
            // update mode machine
            if (mode_machine_ != low_state.mode_machine()) {
                if (mode_machine_ == 0) std::cout << "G1 type: " << unsigned(low_state.mode_machine()) << std::endl;
                mode_machine_ = low_state.mode_machine();
            }
        }

    private:
        inline uint32_t Crc32Core(uint32_t *ptr, uint32_t len) {
            uint32_t xbit = 0;
            uint32_t data = 0;
            uint32_t CRC32 = 0xFFFFFFFF;
            const uint32_t dwPolynomial = 0x04c11db7;

            for (uint32_t i = 0; i < len; i++) {
                xbit = 1 << 31;
                data = ptr[i];
                for (uint32_t bits = 0; bits < 32; bits++) {
                if (CRC32 & 0x80000000) {
                    CRC32 <<= 1;
                    CRC32 ^= dwPolynomial;
                } else
                    CRC32 <<= 1;
                if (data & xbit) CRC32 ^= dwPolynomial;

                xbit >>= 1;
                }
            }

            return CRC32;
        };

        enum class AnkleMode {
            PR = 0,  // Series Control for Ptich/Roll Joints
            AB = 1   // Parallel Control for A/B Joints
        };

        std::string network_interface_name_;
        unitree::robot::ChannelPublisherPtr<LowCmd_> lowcmd_publisher_;
        ChannelSubscriberPtr<LowState_> lowstate_subscriber_;
        std::shared_ptr<unitree::robot::b2::MotionSwitcherClient> mode_switch_manager_;
        AnkleMode mode_pr_;
        uint8_t mode_machine_;

        std::string name_;
        size_t num_motors_;
        std::vector<std::string> joint_names_;

        // ---------- Gains ---------- //
        std::vector<double> kp_;
        std::vector<double> kd_;

        // ---------- Robot Specific ---------- //
        // G1
        static constexpr int G1_MOTOR_NUM = 29;
        // TODO: Fill in
        const std::array<std::string, G1_MOTOR_NUM> G1_JOINT_NAMES = {
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
            "waist_roll_joint",
            "waist_pitch_joint",
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
            "right_wrist_yaw_joint"
        };

        enum G1JointIndex {
            LeftHipPitch = 0,
            LeftHipRoll = 1,
            LeftHipYaw = 2,
            LeftKnee = 3,
            LeftAnklePitch = 4,
            LeftAnkleB = 4,
            LeftAnkleRoll = 5,
            LeftAnkleA = 5,
            RightHipPitch = 6,
            RightHipRoll = 7,
            RightHipYaw = 8,
            RightKnee = 9,
            RightAnklePitch = 10,
            RightAnkleB = 10,
            RightAnkleRoll = 11,
            RightAnkleA = 11,
            WaistYaw = 12,
            WaistRoll = 13,        // NOTE INVALID for g1 23dof/29dof with waist locked
            WaistA = 13,           // NOTE INVALID for g1 23dof/29dof with waist locked
            WaistPitch = 14,       // NOTE INVALID for g1 23dof/29dof with waist locked
            WaistB = 14,           // NOTE INVALID for g1 23dof/29dof with waist locked
            LeftShoulderPitch = 15,
            LeftShoulderRoll = 16,
            LeftShoulderYaw = 17,
            LeftElbow = 18,
            LeftWristRoll = 19,
            LeftWristPitch = 20,   // NOTE INVALID for g1 23dof
            LeftWristYaw = 21,     // NOTE INVALID for g1 23dof
            RightShoulderPitch = 22,
            RightShoulderRoll = 23,
            RightShoulderYaw = 24,
            RightElbow = 25,
            RightWristRoll = 26,
            RightWristPitch = 27,  // NOTE INVALID for g1 23dof
            RightWristYaw = 28     // NOTE INVALID for g1 23dof
        };

        // Go2

    };
} // namespace obelisk