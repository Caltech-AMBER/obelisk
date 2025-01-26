# pragma once
#include "obelisk_robot.h"
#include "obelisk_ros_utils.h"
#include "obelisk_sensor_msgs/msg/obk_joint_encoders.h"
#include "obelisk_sensor_msgs/msg/obk_imu.h"
#include "obelisk_control_msgs/msg/pd_feed_forward.h"

// DDS
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

// IDL
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

namespace obelisk {
    using unitree_control_msg = obelisk_control_msgs::msg::PDFeedForward;

    using namespace unitree::common;
    using namespace unitree::robot;

    /**
     * @brief Obelisk robot interface for the Unitree robots
     */
    class ObeliskUnitreeInterface : public ObeliskRobot<unitree_control_msg> {

    public:
        ObeliskUnitreeInterface(const std::string& node_name)
            : ObeliskRobot<unitree_control_msg>(node_name) {

            // Get network interface name as a parameter
            this->declare_parameter<std::string>("network_interface_name", "");
            network_interface_name_ = this->get_parameter("network_interface_name").as_string();
            RCLCPP_INFO_STREAM(this->get_logger(), "Network interface: " << network_interface_name_);

            // Additional Publishers
            this->RegisterObkPublisher<obelisk_sensor_msgs::msg::ObkJointEncoders>("pub_sensor_setting", "joint_state_pub");
            this->RegisterObkPublisher<obelisk_sensor_msgs::msg::ObkImu>("pub_imu_setting", "imu_state_pub");

            // ---------- Default PD gains ---------- //
            this->declare_parameter<std::vector<double>>("default_kp");
            this->declare_parameter<std::vector<double>>("default_kd");
            kp_ = this->get_parameter("default_kp").as_double_array();
            kd_ = this->get_parameter("default_kd").as_double_array();

            // TODO: User defined safety maybe
            
            ChannelFactory::Instance()->Init(0, network_interface_name_);

            // Try to shutdown motion control-related service
            mode_switch_manager_ = std::make_shared<unitree::robot::b2::MotionSwitcherClient>();
            mode_switch_manager_->SetTimeout(5.0f);
            mode_switch_manager_->Init();
            std::string form, name;
            while (mode_switch_manager_->CheckMode(form, name), !name.empty()) {
                if (mode_switch_manager_->ReleaseMode())
                    std::cout << "Failed to switch to Release Mode\n";
                sleep(5);
            }
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn virtual on_activate(
            const rclcpp_lifecycle::State& prev_state) {
            this->ObeliskRobot::on_activate(prev_state);

            CreateUnitreeSubscribers();

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

    protected:
        virtual void CreateUnitreeSubscribers() = 0;
        virtual void CreateUnitreePublishers() = 0;


        void VerifyParameters() {
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
        }

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

        // ---------- Topics ---------- //
        // TODO: Verify these are the same of the G1 and the Go2
        std::string CMD_TOPIC_;
        std::string STATE_TOPIC_;

        std::string network_interface_name_;
        std::shared_ptr<unitree::robot::b2::MotionSwitcherClient> mode_switch_manager_;

        size_t num_motors_;
        std::vector<std::string> joint_names_;

        // ---------- Gains ---------- //
        std::vector<double> kp_;
        std::vector<double> kd_;
    private:
    };
} // namespace obelisk