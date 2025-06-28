
#include "unitree_interface.h"

// Message Types
#include <msg/ArmString_.hpp>
#include <msg/PubServoInfo_.hpp>
#include <msg/SetServoAngle_.hpp>
#include <msg/SetServoDumping_.hpp>

namespace obelisk {

    using namespace unitree_arm::msg::dds_;

    class D1Interface : public OebliskUnitreeInterface {
    public:
        D1Interface(const std::string& node_name)
        : ObeliskUnitreeInterface(node_name) {
            // Handle joint names
            num_motors_ = D1_MOTOR_NUM;
            for (size_t i = 0; i < num_motors_; i++) {
                joint_names_.push_back(D1_JOINT_NAMES[i]);
            }

            // Define topic names
            CMD_TOPIC_ = "rt/arm_Command"; // commanded servo angles in degrees
            FEEDBACK_TOPIC_ = "rt/arm_Feedback"; // TODO: figure out what this is
            SERVO_TOPIC_ = "rt/current_servo_angle"; // current servo angles in degrees

            // Verify params and create unitree publishers
            VerifyParameters();
            CreateUnitreePublishers();
        }
    protected:
        void CreateUnitreePublishers() override {
            lowcmd_publisher_.reset(new ChannelPublisher<ArmString_>(CMD_TOPIC_));
            lowcmd_publisher_->InitChannel();

            RCLCPP_INFO_STREAM(this->get_logger(), "D1 command publishers created.");
        }

        void CreateUnitreeSubscribers() override {
            
        }

        ChannelPublisherPtr<ArmString_> lowcmd_publisher_;
        ChannelSubscriberPtr<PubServoInfo_> servo_subscriber;

    }
} // namespace obelisk