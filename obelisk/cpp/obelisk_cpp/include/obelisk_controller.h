#include "obelisk_node.h"

namespace obelisk {

template <typename ControlMessageT, typename EstimatorMessageT>
class ObeliskController : public ObeliskNode {
  public:
    explicit ObeliskController(const std::string& name) : ObeliskNode(name) {
        // Declare all paramters
        // TODO: Change defaults to nothing
        this->declare_parameter<std::string>("timer_ctrl_config_str",
                                             "timer_period_sec:1");
        this->declare_parameter<std::string>("pub_ctrl_config_str",
                                             "topic:topic1");
        this->declare_parameter<std::string>("sub_est_config_str",
                                             "topic:topic2");
        // "topic:topic1,message_type:PositionSetpoint"
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State& prev_state) {
        ObeliskNode::on_configure(prev_state);

        // Create the publishers, subscribers, and timers
        control_publisher_ = CreatePublisherFromConfigStr<ControlMessageT>(
            this->get_parameter("pub_ctrl_config_str").as_string());

        state_estimator_subscriber_ =
            CreateSubscriptionFromConfigStr<EstimatorMessageT>(
                this->get_parameter("sub_est_config_str").as_string(),
                std::bind(&ObeliskController::UpdateXHat, this,
                          std::placeholders::_1));

        control_timer_ = CreateWallTimerFromConfigStr(
            this->get_parameter("timer_ctrl_config_str").as_string(),
            std::bind(&ObeliskController::ComputeControl, this));

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
            CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State& prev_state) {
        control_publisher_->on_activate();
        // TODO: Do anything with the timer?

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
            CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State& prev_state) {
        control_publisher_->on_deactivate();
        // TODO: Do anything with the timer?

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
            CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State& prev_state) {
        // Release the shared pointers
        control_publisher_.reset();
        state_estimator_subscriber_.reset();
        control_timer_.reset();

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
            CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State& prev_state) {
        // Release the shared pointers
        control_publisher_.reset();
        state_estimator_subscriber_.reset();
        control_timer_.reset();

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
            CallbackReturn::SUCCESS;
    }

  protected:
    // Abstract callback to be implemented by the user
    virtual ControlMessageT ComputeControl() = 0;

    // Abstract callback to be implemented by the user
    virtual void UpdateXHat(const EstimatorMessageT& msg) = 0;

    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<ControlMessageT>>
        control_publisher_;
    typename rclcpp::Subscription<EstimatorMessageT>::SharedPtr
        state_estimator_subscriber_;
    rclcpp::TimerBase::SharedPtr control_timer_;

  private:
};
} // namespace obelisk
