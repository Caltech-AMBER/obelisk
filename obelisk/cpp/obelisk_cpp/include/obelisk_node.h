#pragma once
#include <variant>

#include "rcl_interfaces/msg/parameter_event.hpp"
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
            bool enable_communication_interface = true)
            : LifecycleNode(
                  node_name, options, enable_communication_interface) {};

        ObeliskNode(const std::string& node_name, const std::string& namespace_,
            const rclcpp::NodeOptions& options  = rclcpp::NodeOptions(),
            bool enable_communication_interface = true)
            : LifecycleNode(node_name, namespace_, options,
                  enable_communication_interface) {};

        template <typename MessageT, typename AllocatorT = std::allocator<void>>
        std::shared_ptr<
            rclcpp_lifecycle::LifecyclePublisher<MessageT, AllocatorT>>
        create_publisher(const std::string& topic_name, const rclcpp::QoS& qos,
            const rclcpp::PublisherOptionsWithAllocator<AllocatorT>& options =
                (rclcpp_lifecycle::create_default_publisher_options<
                    AllocatorT>())) {
            if (ValidMessage<MessageT, ObeliskMsgs>::value) {
                return rclcpp_lifecycle::LifecycleNode::create_publisher<
                    MessageT, AllocatorT>(topic_name, qos, options);
            }

            throw std::runtime_error(
                "Provided message type is not a valid Obelisk message!");
        };

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
                (MessageMemoryStrategyT::create_default())) {
            return rclcpp_lifecycle::LifecycleNode::create_subscription<
                MessageT, CallbackT, AllocatorT, SubscriptionT,
                MessageMemoryStrategyT>(topic_name, qos, callback, options);
        };

        template <typename scalar, typename scalar_2>
        scalar SimpleFunc(scalar a, scalar_2 b) {
            return a;
        }

       private:
        using ObeliskMsgs    = std::tuple<rcl_interfaces::msg::ParameterEvent>;
        using ROSAllowedMsgs = std::tuple<rcl_interfaces::msg::ParameterEvent>;

        // Helper functions for determining if we have a valid message
        template <typename T, typename Tuple>
        struct has_type;

        template <typename T>
        struct has_type<T, std::tuple<>> : std::false_type {};

        template <typename T, typename U, typename... Ts>
        struct has_type<T, std::tuple<U, Ts...>>
            : has_type<T, std::tuple<Ts...>> {};

        template <typename T, typename... Ts>
        struct has_type<T, std::tuple<T, Ts...>> : std::true_type {};

        template <typename T, typename Tuple>
        using ValidMessage = typename has_type<T, Tuple>::type;
    };
}  // namespace obelisk
