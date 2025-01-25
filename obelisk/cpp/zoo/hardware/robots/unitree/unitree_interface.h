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

    static const std::string HG_CMD_TOPIC = "rt/lowcmd";
    static const std::string HG_IMU_TORSO = "rt/secondary_imu";
    static const std::string HG_STATE_TOPIC = "rt/lowstate";

    using namespace unitree::common;
    using namespace unitree::robot;
    using namespace unitree_hg::msg::dds_;

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

            RCLCPP_INFO_STREAM(this->get_logger(), "Network interface: " << network_interface_name_);

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
            RCLCPP_INFO_STREAM(this->get_logger(), "Applying control to Unitree robot!");

            LowCmd_ dds_low_command;
            dds_low_command.mode_pr() = static_cast<uint8_t>(mode_pr_);
            dds_low_command.mode_machine() = mode_machine_;

            // const std::shared_ptr<const MotorCommand> mc = motor_command_buffer_.GetData();

            double time_sec = this->get_clock()->now().seconds();
            double ankle_pos = 0.3*3.14*sin(3.14*2*time_sec);

            const int G1_MOTOR_NUM = 29;
            for (size_t i = 0; i < G1_MOTOR_NUM; i++) {
                if (i == 10) {
                    dds_low_command.motor_cmd().at(i).mode() = 1;  // 1:Enable, 0:Disable
                    dds_low_command.motor_cmd().at(i).tau() = 0;
                    dds_low_command.motor_cmd().at(i).q() = ankle_pos;
                    dds_low_command.motor_cmd().at(i).dq() = 0;
                    dds_low_command.motor_cmd().at(i).kp() = 40;
                    dds_low_command.motor_cmd().at(i).kd() = 1;
                }
                RCLCPP_INFO_STREAM(this->get_logger(), "[" << i << "]" << "data: " 
                    << dds_low_command.motor_cmd().at(i).tau() << ", "
                    << dds_low_command.motor_cmd().at(i).q() << ", "
                    << dds_low_command.motor_cmd().at(i).dq() << ", "
                    << dds_low_command.motor_cmd().at(i).kp() << ", "
                    << dds_low_command.motor_cmd().at(i).kd() << ", ");
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
    };
} // namespace obelisk