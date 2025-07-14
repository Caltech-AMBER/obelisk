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
#include <d1/msg/ArmString_.hpp>
#include <d1/utils/AllServoFeedback.hpp>
#include <d1/utils/AllServoCtrl.hpp>
#include <d1/utils/Unit.hpp>

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
                    PUB_SENSOR_SETTING_NAME, PUB_SENSOR_SETTING_KEY
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

                        if (seq == AllServoFeedback::SEQ 
                            && address == AllServoFeedback::ADDRESS
                            && funcode == AllServoFeedback::FUNCODE) {
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
                Publishes joint1 to joint6 positions in radians as well as 
                gripper1 and gripper2 in meters to the Obelisk 
                `PUB_SENSOR_SETTING_KEY` topic. Velocities (default to 0 rad/s). 

                document (Document): arm feedback document that contains 
                joint1 to joint6 positions and gripper position in degrees.
                One servo motor controls gripper1 and gripper2, so the 
                gripper position is the position of this servo motor.

                The joint state will include joint and gripper data for ease
                of publishing arm data.
                */
                d1_sensor_msg joint_state;
                joint_state.header.stamp = this->now();
                joint_state.joint_pos.resize(NUM_CONTROL_INPUTS_SIM);
                // arm_Feedback doesn't report joint velocities, so default
                // them to 0.
                joint_state.joint_vel.resize(NUM_CONTROL_INPUTS_SIM, 0.0);
                joint_state.joint_names.resize(NUM_CONTROL_INPUTS_SIM);
                
                std::string joint_name_hardware;
                std::string joint_name;
                double joint_pos_hardware;
                double joint_pos;

                // Populate data for joint1 to 6.
                int i;
                for (i = 0; i < NUM_JOINTS; i++) {
                    joint_name_hardware = JOINT_NAMES_HARDWARE.at(i);
                    joint_name = JOINT_NAMES.at(i);
                    joint_state.joint_names.at(i) = joint_name;

                    joint_pos_hardware = document["data"][joint_name_hardware.c_str()].GetDouble();

                    // Convert the hardware joint positions to the proper units
                    if (JOINT_UNITS_HARDWARE == DEGREES && JOINT_UNITS == RADIANS) {
                        joint_pos = joint_pos_hardware * M_PI / 180;
                    }
                    else {
                        RCLCPP_ERROR_STREAM(
                            this->get_logger(),
                            "Didn't implement unit conversion yet."
                        );
                        return;
                    }
                    joint_state.joint_pos.at(i) = joint_pos;
                }
                // Set the gripper1 state
                joint_state.joint_names.at(i) = GRIPPER_NAMES.at(0);
                double gripper_pos_hardware = document["data"][GRIPPER_NAME_HARDWARE.c_str()].GetDouble();
                double gripper_pos = degrees2meters(gripper_pos_hardware);
                joint_state.joint_pos.at(i) = gripper_pos;

                // Set gripper2 state
                joint_state.joint_names.at(i + 1) = GRIPPER_NAMES.at(1);
                joint_state.joint_pos.at(i + 1) = -gripper_pos;

                this->GetPublisher<d1_sensor_msg>(PUB_SENSOR_SETTING_KEY)->publish(joint_state);
            }

            void ApplyControl(const d1_control_msg& control_msg) {
                /* 
                Extract joint1 to joint6 and gripper1 to gripper2 to command to 
                robot.
                joint1 to joint6 positions are in radians.
                gripper1 to gripper2 positions are in meters.

                The commanded message must contain
                joint1 to joint6 positions and the gripper position in degrees.
                One servo motor controls gripper1 and gripper2, so the 
                gripper position is the position of this servo motor.
                */
                std::vector<double> q_des = control_msg.q_des;
                std::vector<double> qd(
                    q_des.begin(), 
                    q_des.begin() + NUM_JOINTS
                );
                double gripper_pos = -q_des.back();
                AllServoCtrl ctrl = { 
                    qd, 
                    gripper_pos,
                    JOINT_NAMES_HARDWARE,
                    GRIPPER_NAME_HARDWARE,
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

            const std::string PUB_SENSOR_SETTING_NAME = "pub_sensor_setting";
            const std::string PUB_SENSOR_SETTING_KEY = "joint_state_pub";
            
            const std::string CMD_TOPIC = "rt/arm_Command";
            const std::string STATE_TOPIC = "rt/arm_Feedback";
            

            static const int NUM_JOINTS = 6;
            static const int NUM_CONTROL_INPUTS_HARDWARE = 7; 
            static const int NUM_CONTROL_INPUTS_SIM = 8; 

            /*
            Software parameters:
            --------------------
            These parameters are used in the D1 interface, controller,
            estimator, and robot models.  
            */
            const std::vector<std::string> JOINT_NAMES = {
                "joint1", "joint2", "joint3", "joint4", 
                "joint5", "joint6", 
            };
            const std::vector<std::string> GRIPPER_NAMES = {
                "gripper1", "gripper2",
            };
            const Unit JOINT_UNITS = RADIANS;

            /* 
            Hardware parameters:
            --------------------
            These are used when sending and processing messages directly
            from the D1 arm. 
            These differ from the naming convention of the Unitree Z1 Arm. 
            Since the Z1 Arm has an SDK for joystick control, we'll use the Z1 
            Arm naming convention. 
            */
            const std::vector<std::string> JOINT_NAMES_HARDWARE = {
                "angle0", "angle1", "angle2", "angle3", 
                "angle4", "angle5",
            };
            const std::string GRIPPER_NAME_HARDWARE = "angle6";
            const Unit JOINT_UNITS_HARDWARE = DEGREES;
    };
} // namespace obelisk