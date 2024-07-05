#include "rclcpp/rclcpp.hpp"

#include "obelisk_estimator.h"
#include "obelisk_ros_utls.h"

class JointEncodersPassthrough : public obelisk::ObeliskEstimator<obelisk_estimator_msgs::msg::EstimatedState> {
  public:
    JointEncodersPassthrough(const std::string& name)
        : obelisk::ObeliskEstimator<obelisk_estimator_msgs::msg::EstimatedState>(name) {
        this->declare_parameter<std::string>("sub_sensor_setting", "");
    }

  protected:
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State& prev_state) {
        this->ObeliskEstimator<obelisk_estimator_msgs::msg::EstimatedState>::on_configure(prev_state);

        sensor_sub_ = this->ObeliskNode::CreateSubscriptionFromConfigStr<obelisk_sensor_msgs::msg::JointEncoders>(
            this->get_parameter("sub_sensor_setting").as_string(),
            std::bind(&JointEncodersPassthrough::JointEncoderCallback, this, std::placeholders::_1));

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    void JointEncoderCallback(const obelisk_sensor_msgs::msg::JointEncoders& msg) { joint_encoders_ = msg.y; }

    obelisk_estimator_msgs::msg::EstimatedState ComputeStateEstimate() override {
        obelisk_estimator_msgs::msg::EstimatedState msg;

        msg.x_hat = joint_encoders_;

        this->estimator_publisher_->publish(msg);

        return msg;
    };

  private:
    std::vector<double> joint_encoders_;
    typename rclcpp::Subscription<obelisk_sensor_msgs::msg::JointEncoders>::SharedPtr sensor_sub_;
};

int main(int argc, char* argv[]) {
    obelisk::utils::SpinObelisk<JointEncodersPassthrough, rclcpp::executors::MultiThreadedExecutor>(
        argc, argv, "passthrough_estimator");
}
