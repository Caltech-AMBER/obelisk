#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

/**
 * ObeliskNode class.
 * A lifecycle node whose publishers and subscribers can only publish and
 * subscribe to Obelisk messages.
 */

namespace obelisk {
    class ObeliskNode : public rclcpp_lifecycle::LifecycleNode {
       public:
        explicit ObeliskNode(const std::string& node_name,
            const rclcpp::NodeOptions& options  = rclcpp::NodeOptions(),
            bool enable_communication_interface = true);

        ObeliskNode(const std::string& node_name, const std::string& namespace_,
            const rclcpp::NodeOptions& options  = rclcpp::NodeOptions(),
            bool enable_communication_interface = true);

        template <typename MessageT, typename AllocatorT = std::allocator<void>>
        std::shared_ptr<
            rclcpp_lifecycle::LifecyclePublisher<MessageT, AllocatorT>>
        create_publisher(const std::string& topic_name, const rclcpp::QoS& qos,
            const rclcpp::PublisherOptionsWithAllocator<AllocatorT>& options =
                (rclcpp_lifecycle::create_default_publisher_options<
                    AllocatorT>()));

        template <typename MessageT, typename CallbackT,
            typename AllocatorT    = std::allocator<void>,
            typename SubscriptionT = rclcpp::Subscription<MessageT, AllocatorT>,
            typename MessageMemoryStrategyT =
                typename SubscriptionT::MessageMemoryStrategyType>
        std::shared_ptr<SubscriptionT> create_subscription(
            const std::string& topic_name, const rclcpp::QoS& qos,
            CallbackT&& callback,
            const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>&
                options = rclcpp_lifecycle::create_default_subscription_options<
                    AllocatorT>(),
            typename MessageMemoryStrategyT::SharedPtr msg_mem_strat =
                (MessageMemoryStrategyT::create_default()));
    };

}  // namespace obelisk
