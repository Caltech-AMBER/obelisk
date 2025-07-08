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
#include <msg/AllJointCtrl.hpp>
#include <msg/Unit.hpp>

namespace obelisk {
    using namespace unitree::common;
    using namespace unitree::robot;
    using namespace unitree_arm::msg::dds_;
    using namespace rapidjson;
    
    using d1_control_msg = obelisk_control_msgs::msg::PositionSetpoint;
    using d1_sensor_msg = obelisk_sensor_msgs::msg::ObkJointEncoders;
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class D1Interface : public ObeliskRobot<d1_control_msg> {
        public:
            D1Interface(const std::string& node_name)
                : ObeliskRobot<d1_control_msg>(node_name) {
                // Register additional publishers
                this->RegisterObkPublisher<d1_sensor_msg>(
                    PUB_SENSOR_SETTING_TOPIC, PUB_SENSOR_SETTING_KEY
                );

                // Get network interface name as a parameter
                this->declare_parameter<std::string>("network_interface_name", "");
                network_interface_name_ = this->get_parameter(
                    "network_interface_name").as_string();
                RCLCPP_INFO_STREAM(this->get_logger(), "Network interface: " 
                << network_interface_name_);
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
                lowstate_subscriber_.reset(new ChannelSubscriber<ArmString_>(STATE_TOPIC));
                lowstate_subscriber_->InitChannel(
                    std::bind(&D1Interface::LowStateHandler, 
                    this, std::placeholders::_1)
                );
                RCLCPP_INFO_STREAM(this->get_logger(), "Subscribed to " << STATE_TOPIC);
            }

            CallbackReturn virtual on_activate(
                const rclcpp_lifecycle::State& prev_state) {
                this->ObeliskRobot::on_activate(prev_state);
                CreateSubscribers();
                return CallbackReturn::SUCCESS;
            }

            void LowStateHandler(const void* msg) {
                /*
                Called when the arm_Feedback subscriber hears a message sent by 
                the robot over the state topic. Parses the message to determine 
                if it can be used to publish a joint state.
                */
                const ArmString_* state = (const ArmString_*) msg;
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
                            PublishJointState(document);
                        }
                    }
                    else {
                        RCLCPP_ERROR_STREAM(this->get_logger(), 
                        "Unknown arm feedback object");
                    }
                }
                else {
                    RCLCPP_ERROR_STREAM(this->get_logger(), 
                    "Failed to parse arm feedback JSON message");
                }
            }

            void PublishJointState(const Document& document) {
                /* 
                Publishes joint0 to joint5 positions in radians as well as 
                joint6 and joint6_2 in meters to the Obelisk 
                `PUB_SENSOR_SETTING_KEY` topic. Velocities (default to 0 rad/s). 

                document (Document): arm feedback document that contains 
                joint0 to joint5 positions in degrees and 
                joint6 position in millimeters. This joint6 position is double
                of what it is in the simulation.
                */
                d1_sensor_msg joint_state;
                joint_state.header.stamp = this->now();
                joint_state.joint_pos.resize(NUM_JOINTS);
                joint_state.joint_vel.resize(NUM_JOINTS);
                joint_state.joint_names.resize(NUM_JOINTS);
                
                std::string joint_name_hardware;
                std::string joint_name;
                Unit joint_unit_hardware;
                Unit joint_unit;
                double joint_pos_hardware;
                double joint_pos;

                for (int i = 0; i < NUM_JOINTS; i++) {
                    joint_name_hardware = JOINT_NAMES_HARDWARE.at(i);
                    joint_name = JOINT_NAMES.at(i);
                    joint_state.joint_names.at(i) = joint_name;

                    joint_unit_hardware = JOINT_UNITS_HARDWARE.at(i);
                    joint_unit = JOINT_UNITS.at(i);

                    joint_pos_hardware = document["data"][joint_name_hardware.c_str()].GetDouble();

                    // Convert the hardware joint positions to the proper units
                    if (joint_unit_hardware == DEGREES && joint_unit == RADIANS) {
                        joint_pos = joint_pos_hardware * M_PI / 180;
                    }
                    else if (joint_unit_hardware == MILLIMETERS && joint_unit == METERS) {
                        joint_pos = joint_pos_hardware / 1000;
                        joint_pos /= 2;
                    }
                    else {
                        RCLCPP_ERROR_STREAM(
                            this->get_logger(),
                            "Didn't implement unit conversion yet."
                        );
                        return;
                    }
                    joint_state.joint_pos.at(i) = joint_pos;

                    // Set joint6 position to be the negative of the 
                    // joint6_2 position.
                    if (i == NUM_JOINTS - 1) {
                        joint_state.joint_pos.at(i) = -joint_state.joint_pos.at(i - 1);
                    }
                    
                    // arm_Feedback doesn't report joint velocities, so default
                    // to 0.
                    joint_state.joint_vel.at(i) = 0; 
                }

                this->GetPublisher<d1_sensor_msg>(PUB_SENSOR_SETTING_KEY)->publish(joint_state);
            }

            void ApplyControl(const d1_control_msg& control_msg) {
                /* 
                Extract joint0 to joint6 positions to command to robot.
                joint0 to joint5 positions are in radians.
                joint6 position is in meters and is half of what it is according
                to the hardware.

                The commanded message must contain
                joint0 to joint5 positions are in degrees and
                joint6 position in millimeters.
                */
                std::vector<double> q_des = control_msg.q_des;
                std::vector<double> q_des_7(
                    q_des.begin(), 
                    q_des.begin() + NUM_JOINTS - 1
                );
                std::vector<std::string> joint_names_7(
                    JOINT_NAMES_HARDWARE.begin(), 
                    JOINT_NAMES_HARDWARE.begin() + NUM_JOINTS - 1
                );

                AllJointCtrl ctrl = { 
                    q_des_7, 
                    joint_names_7,
                    JOINT_UNITS_HARDWARE,
                    JOINT_UNITS
                };
                std::string ctrl_str = ctrl.str();
            
                // Create message
                ArmString_ msg{};
                msg.data_() = ctrl_str;
                
                // Publish message
                lowcmd_publisher_->Write(msg);
            }

        private:
            std::string network_interface_name_;
            ChannelPublisherPtr<ArmString_> lowcmd_publisher_;
            ChannelSubscriberPtr<ArmString_> lowstate_subscriber_;

            const std::string PUB_SENSOR_SETTING_TOPIC = "pub_sensor_setting";
            const std::string PUB_SENSOR_SETTING_KEY = "joint_state_pub";
            
            const std::string CMD_TOPIC = "rt/arm_Command";
            const std::string STATE_TOPIC = "rt/arm_Feedback";
            

            const int NUM_JOINTS = 8;
            const std::vector<std::string> JOINT_NAMES = {
                "joint0", "joint1", "joint2", "joint3", 
                "joint4", "joint5", "joint6", "joint6_2"
            };
            /* Below are the joint names the hardware uses, 
            but they are confusing because joint 6 and joint 6_2 are not angles.
            */
            const std::vector<std::string> JOINT_NAMES_HARDWARE = {
                "angle0", "angle1", "angle2", "angle3", 
                "angle4", "angle5", "angle6", "angle6_2"
            };
            const std::vector<Unit> JOINT_UNITS = {
                RADIANS, RADIANS, RADIANS, RADIANS,
                RADIANS, RADIANS, METERS, METERS
            };
            const std::vector<Unit> JOINT_UNITS_HARDWARE = {
                DEGREES, DEGREES, DEGREES, DEGREES,
                DEGREES, DEGREES, MILLIMETERS, MILLIMETERS
            };
    };
} // namespace obelisk