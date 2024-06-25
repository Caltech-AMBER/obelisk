#include "obelisk_node.h"

namespace obelisk {
// TODO: Consider multiple types here (variadic)
template <typename ControlMessageT, typename StateMessageT>
class ObeliskController : public ObeliskNode {
  public:
    explicit ObeliskController(const std::string& name)
        : ObeliskNode(name), control_publisher_(nullptr),
          state_estimator_subscriber_(nullptr) {
        // Declare all paramters
        this->declare_parameter("callback_group_config_strs", "topic:topc1");
        this->declare_parameter("timer_ctrl_config_str", "topic:topc1");
        this->declare_parameter("pub_ctrl_config_str", "topic:topic1");
        this->declare_parameter("sub_est_config_str", "topic:topic1");
    }

    template <typename CallbackT>
    void CreateStateEstimatorSubscriber(CallbackT&& callback) {
        // Create the state estimator subscriber with the proper configuration
        state_estimator_subscriber_ =
            CreateSubscriptionFromConfigStr<StateMessageT>(
                this->get_parameter("sub_est_config_str"), std::move(callback));
    };

    void CreateControllerPublisher() {
        // Create the state estimator subscriber with the proper configuration
        control_publisher_ = CreatePublisherFromConfigStr<ControlMessageT>(
            this->get_parameter("pub_ctrl_config_str"));
    };

    // TODO: What does the timer take?
    template <
        typename MessageT, typename AllocatorT = std::allocator<void>,
        typename SubscriptionT = rclcpp::Subscription<MessageT, AllocatorT>>
    std::shared_ptr<SubscriptionT> CreateControlTimer() {
        // Create the state estimator subscriber with the proper configuration
        return CreatePublisherFromConfigStr<MessageT>(
            this->get_parameter("timer_ctrl_config_str"));
    };

  protected:
    typename rclcpp::Publisher<ControlMessageT>::SharedPtr control_publisher_;
    typename rclcpp::Subscription<StateMessageT>::SharedPtr
        state_estimator_subscriber_;

  private:
};
} // namespace obelisk
