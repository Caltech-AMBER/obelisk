#include <cmath>
#include <rapidjson/document.h>

// Obelisk
#include "obelisk_robot.h"
#include "obelisk_ros_utils.h"
#include "obelisk_sensor_msgs/msg/obk_joint_encoders.h"

// DDS
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

// D1 Arm Message Types
#include <msg/ArmString_.hpp>
#include <msg/AllJointFeedback.hpp>


namespace obelisk {
    using namespace unitree::common;
    using namespace unitree::robot;
    using namespace unitree_arm::msg::dds_;
    
    using d1_control_msg = obelisk_control_msgs::msg::PositionSetpoint;
    using d1_sensor_msg = obelisk_sensor_msgs::msg::ObkJointEncoders;
    using Document = rapidjson::Document;

    class D1Interface : public ObeliskRobot<d1_control_msg> {
        public:
            D1Interface(const std::string& node_name)
                : ObeliskRobot<d1_control_msg>(node_name) {
                // Register additional publishers
                this->RegisterObkPublisher<d1_sensor_msg>("pub_sensor_setting", pub_joint_state_key_);

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
                // Create low state subscriber after the publisher(s) have been activated
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
                const ArmString_* state = (const ArmString_*) msg;
                // RCLCPP_INFO_STREAM(this->get_logger(), "arm_Feedback_data:" << state->data_());

                std::string json_str = state->data_();

                Document document;
                document.Parse(json_str.c_str());

                if (!document.HasParseError()) {
                    if (document.IsObject()) {
                        int seq = document["seq"].GetInt();
                        int address = document["address"].GetInt();
                        int funcode = document["funcode"].GetInt();

                        if (seq == AllJointFeedback::SEQ 
                            && address == AllJointFeedback::ADDRESS
                            && funcode == AllJointFeedback::FUNCODE) {
                            RCLCPP_INFO_STREAM(this->get_logger(), "Publishing arm feedback data");
                            PublishJointState(document);
                        }
                    }
                    else {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "Unknown arm feedback object");
                    }
                }
                else {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to parse arm feedback JSON message");
                }
            }

            void PublishJointState(const Document& document) {
                // Joints
                d1_sensor_msg joint_state;
                joint_state.header.stamp = this->now();
                joint_state.joint_pos.resize(NUM_JOINTS);
                joint_state.joint_vel.resize(NUM_JOINTS);
                joint_state.joint_names.resize(NUM_JOINTS);
                
                std::string joint_name;
                for (int i = 0; i < NUM_JOINTS; i++) {
                    joint_name = JOINT_NAMES[i];
                    joint_state.joint_names.at(i) = joint_name;
                    joint_state.joint_pos.at(i) = document["data"][joint_name.c_str()].GetDouble();
                    joint_state.joint_vel.at(i) = 0; // arm_Feedback doesn't report joint velocities.
                }

                this->GetPublisher<d1_sensor_msg>(pub_joint_state_key_)->publish(joint_state);
            }

            void ApplyControl(const d1_control_msg& control_msg) {
                /*
                Command the joint positions (in degrees) of the D1 Arm.

                The D1 Arm can also be commanded to release control of all motors, but this is not implemented here.
                */
                // FIXME: Clean code, add visualization in yaml
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
                    cmd << ",\"" << JOINT_NAMES[i] << "\":" << pos_in_degrees;
                }
                
                cmd << "}}";
                std::string cmd_str = cmd.str();

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
        const std::string pub_joint_state_key_ = "joint_state_pub";

        static const int NUM_JOINTS = 7;
        const std::array<std::string, NUM_JOINTS> JOINT_NAMES = {
            "angle0",
            "angle1",
            "angle2",
            "angle3",
            "angle4",
            "angle5",
            "angle6",
        };
        static const int SEQ_ALL_JOINTS = 4;
        static const int ADDRESS_ALL_JOINTS = 1;
        static const int FUNCODE_ALL_JOINTS = 2;
        static const int MODE_ALL_JOINTS = 1;
    };
}