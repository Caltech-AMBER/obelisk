#include "obelisk_node.h"

namespace obelisk {
class ObeliskController : public ObeliskNode {
  public:
    explicit ObeliskController(const std::string& name) : ObeliskNode(name) {
        // Declare all paramters
        this->declare_parameter("callback_group_config_strs", "topic:topc1");
        this->declare_parameter("timer_ctrl_config_str", "topic:topc1");
        this->declare_parameter("pub_ctrl_config_str", "topic:topic1");
        this->declare_parameter("sub_est_config_str", "topic:topic1");
    }

    template <
        typename MessageT, typename CallbackT,
        typename AllocatorT    = std::allocator<void>,
        typename SubscriptionT = rclcpp::Subscription<MessageT, AllocatorT>>
    std::shared_ptr<SubscriptionT>
    CreateStateEstimatorSubscriber(CallbackT&& callback) {
        // Create the state estimator subscriber with the proper configuration
        return CreateSubscriptionFromConfigStr<MessageT>(
            this->get_parameter("sub_est_config_str"), callback);
    };

    template <
        typename MessageT, typename AllocatorT = std::allocator<void>,
        typename SubscriptionT = rclcpp::Subscription<MessageT, AllocatorT>>
    std::shared_ptr<SubscriptionT> CreateControllerPublisher() {
        // Create the state estimator subscriber with the proper configuration
        return CreatePublisherFromConfigStr<MessageT>(
            this->get_parameter("pub_ctrl_config_str"));
    };

    template <
        typename MessageT, typename AllocatorT = std::allocator<void>,
        typename SubscriptionT = rclcpp::Subscription<MessageT, AllocatorT>>
    std::shared_ptr<SubscriptionT> CreateControlTimer() {
        // Create the state estimator subscriber with the proper configuration
        return CreatePublisherFromConfigStr<MessageT>(
            this->get_parameter("timer_ctrl_config_str"));
    };

  protected:
  private:
};
} // namespace obelisk
