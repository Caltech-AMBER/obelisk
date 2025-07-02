#include <cmath>

#include "obelisk_robot.h"
#include "obelisk_ros_utils.h"
#include "obelisk_sensor_msgs/msg/obk_joint_encoders.h"

// DDS
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

// Message Types
#include <msg/ArmString_.hpp>

namespace obelisk {
    using namespace unitree::common;
    using namespace unitree::robot;
    using namespace unitree_arm::msg::dds_;
    
    using d1_control_msg = obelisk_control_msgs::msg::PositionSetpoint;
    using d1_sensor_msg = obelisk_sensor_msgs::msg::ObkJointEncoders; // TODO: Do something with this?

    class D1Interface : public ObeliskRobot<d1_control_msg> {
        public:
            D1Interface(const std::string& node_name)
                : ObeliskRobot<d1_control_msg>(node_name) {
                RCLCPP_INFO_STREAM(this->get_logger(), "Initializing Interface");
                // Get network interface name as a parameter
                this->declare_parameter<std::string>("network_interface_name", "");
                network_interface_name_ = this->get_parameter("network_interface_name").as_string();
                RCLCPP_INFO_STREAM(this->get_logger(), "Network interface: " << network_interface_name_);

                ChannelFactory::Instance()->Init(0, network_interface_name_);
                
                CreatePublishers();
            }

            void CreatePublishers() {
                // Create low level command publisher
                lowcmd_publisher_.reset(new ChannelPublisher<ArmString_>(CMD_TOPIC));
                lowcmd_publisher_->InitChannel();
            }

            void CreateSubscribers() {
                // Create subscribers
                // Should create after the publishers have been activated
                RCLCPP_INFO_STREAM(this->get_logger(), "Creating subscribers");
                lowstate_subscriber_.reset(new ChannelSubscriber<ArmString_>(STATE_TOPIC));
                lowstate_subscriber_->InitChannel(std::bind(&D1Interface::LowStateHandler, this, std::placeholders::_1));
                RCLCPP_INFO_STREAM(this->get_logger(), "Subscribed to " << STATE_TOPIC);
            }

            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn virtual on_activate(
                const rclcpp_lifecycle::State& prev_state) {
                this->ObeliskRobot::on_activate(prev_state);

                CreateSubscribers();

                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
            }

            void LowStateHandler(const void* msg) {
                /*
                Called when the arm_Feedback subscriber hears a message on the state topic.
                
                The joint positions are in degrees.
                */
                const ArmString_* pm = (const ArmString_*) msg;
                RCLCPP_INFO_STREAM(this->get_logger(), "arm_Feedback_data:" << pm->data_());
            }

            void ApplyControl(const d1_control_msg& control_msg) {
                /*
                Command the joint positions of the D1 Arm.

                The D1 Arm can also be commanded to release control of all motors, but this is not implemented here.
                */
                std::stringstream cmd;
                cmd << "{\"seq\":" 
                << SEQ_ALL_JOINTS 
                << ",\"address\":" 
                << ADDRESS_ALL_JOINTS 
                << ",\"funcode\":" 
                << FUNCODE_ALL_JOINTS 
                << ",\"data\":{\"mode\":"
                << MODE_ALL_JOINTS;

                double pos_in_rads;
                double pos_in_degrees;

                for (int i = 0; i < NUM_JOINTS; i++) {
                    pos_in_rads = control_msg.q_des.at(i);
                    pos_in_degrees = pos_in_rads * 180 / M_PI;
                    // Round to two decimal places
                    pos_in_degrees = std::round(pos_in_degrees * 100) / 100;
                    cmd << ",\"angle" << i << "\":" << pos_in_degrees;
                }
                
                cmd << "}}";
                std::string cmd_str = cmd.str();
                // RCLCPP_INFO_STREAM(this->get_logger(), "cmd_str: " << cmd_str);

                // Create message
                ArmString_ msg{};
                msg.data_() = cmd_str;

                // Publish message
                lowcmd_publisher_->Write(msg);
            }
        
        std::string network_interface_name_;
        ChannelPublisherPtr<ArmString_> lowcmd_publisher_;
        ChannelSubscriberPtr<ArmString_> lowstate_subscriber_;

        const std::string CMD_TOPIC = "rt/arm_Command";
        const std::string STATE_TOPIC = "rt/arm_Feedback";
        
        static const int NUM_JOINTS = 7;
        static const int SEQ_ALL_JOINTS = 4;
        static const int ADDRESS_ALL_JOINTS = 1;
        static const int FUNCODE_ALL_JOINTS = 2;
        static const int MODE_ALL_JOINTS = 1;
    };
}

// #include "unitree_interface.h"

// // Message Types
// #include <msg/ArmString_.hpp>
// #include <msg/PubServoInfo_.hpp>

// namespace obelisk {

//     using namespace unitree_arm::msg::dds_;

//     class D1Interface : public ObeliskUnitreeInterface {
//     public:
//         D1Interface(const std::string& node_name)
//         : ObeliskUnitreeInterface(node_name) {
//             // Handle joint names
//             num_motors_ = D1_MOTOR_NUM;
//             for (size_t i = 0; i < num_motors_; i++) {
//                 joint_names_.push_back(D1_JOINT_NAMES[i]);
//             }

//             // Define topic names
//             CMD_TOPIC_ = "rt/arm_Command"; // commanded servo angles in degrees
//             // FEEDBACK_TOPIC_ = "rt/arm_Feedback"; // TODO: We probably don't need this since we have SERVO_TOPIC_
//             SERVO_TOPIC_ = "rt/current_servo_angle"; // current servo angles in degrees

//             // Verify params and create unitree publishers
//             VerifyParameters();
//             CreateUnitreePublishers();
//         }
//     protected:
//         void CreateUnitreePublishers() override {
//             lowcmd_publisher_.reset(new ChannelPublisher<ArmString_>(CMD_TOPIC_));
//             lowcmd_publisher_->InitChannel();

//             RCLCPP_INFO_STREAM(this->get_logger(), "D1 command publishers created.");
//         }

//         void CreateUnitreeSubscribers() override {
//             lowstate_subscriber_.reset(new ChannelSubscriber<PubServoInfo_>(SERVO_TOPIC_));
//             lowstate_subscriber_->InitChannel(std::bind(&D1Interface::LowStateHandler, this, std::placeholders::_1_), 1)
//         }

//         void ApplyControl(const unitree_control_msg& msg) override {
//             // The Unitree D1 Arm only takes in position commands, however, it's ideal that at the Unitree robots
//             // inherit the UnitreeInterface class and thus use the `unitree_control_msg` type.
//             RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Applying control to D1 Arm!");

//             // Create packet
            

//             // Verify message
//             if (msg.joint_names.size() != msg.)
//         }
//         ChannelPublisherPtr<ArmString_> lowcmd_publisher_;
//         ChannelSubscriberPtr<PubServoInfo_> servo_subscriber;

//     }
// } // namespace obelisk