#include "obelisk_node.h"

namespace obelisk {

    ObeliskNode::ObeliskNode(const std::string& node_name,
        const rclcpp::NodeOptions& options, bool enable_communication_interface)
        : LifecycleNode(node_name, options, enable_communication_interface) {};

    ObeliskNode::ObeliskNode(const std::string& node_name,
        const std::string& namespace_, const rclcpp::NodeOptions& options,
        bool enable_communication_interface)
        : LifecycleNode(node_name, namespace_, options,
              enable_communication_interface) {};

    template <typename MessageT, typename AllocatorT = std::allocator<void>>
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<MessageT, AllocatorT>>
    ObeliskNode::create_publisher(const std::string& topic_name,
        const rclcpp::QoS& qos,
        const rclcpp_lifecycle::PublisherOptionsWithAllocator<AllocatorT>&
            options) {
        // TODO: Check if this is a valid message
        rclcpp_lifecycle::LifecycleNode::create_publisher(
            topic_name, qos, options);
    }

    template <typename MessageT, typename CallbackT,
        typename AllocatorT    = std::allocator<void>,
        typename SubscriptionT = rclcpp::Subscription<MessageT, AllocatorT>,
        typename MessageMemoryStrategyT =
            typename SubscriptionT::MessageMemoryStrategyType>
    std::shared_ptr<SubscriptionT> create_subscription(
        const std::string& topic_name, const rclcpp::QoS& qos,
        CallbackT&& callback,
        const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>& options =
            rclcpp_lifecycle::create_default_subscription_options<AllocatorT>(),
        typename MessageMemoryStrategyT::SharedPtr msg_mem_strat =
            (MessageMemoryStrategyT::create_default())) {
        // TODO: Check if this is a valid message
        rclcpp_lifecycle::LifecycleNode::create_subscription(
            topic_name, qos, callback, options);
    }

}  // namespace obelisk
