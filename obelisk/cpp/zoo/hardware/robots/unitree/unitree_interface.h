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

    /**
     * @brief Obelisk robot interface for the Leap hand hardware
     */
    class ObeliskUnitreeInterface : public ObeliskRobot<unitree_control_msg> {
    public:
        ObeliskUnitreeInterface(const std::string& node_name)
            : ObeliskRobot<unitree_control_msg>(node_name) {

            // Get network interface name as a parameter

            // ChannelFactory::Instance()->Init(0, network_interface_name_);
        }
    protected:

        void ApplyControl(const unitree_control_msg& msg) override {
            // Apply control to the robot
            RCLCPP_INFO_STREAM(this->get_logger(), "Applying control to Unitree robot!");
        }

    private:
        std::string network_interface_name_;
    };
} // namespace obelisk