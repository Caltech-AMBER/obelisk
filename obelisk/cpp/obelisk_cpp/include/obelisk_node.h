#pragma once
#include <tuple>

#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "obelisk_control_msgs/msg/position_setpoint.hpp"
#include "obelisk_estimator_msgs/msg/estimated_state.hpp"
#include "obelisk_sensor_msgs/msg/joint_encoder.hpp"
#include "obelisk_sensor_msgs/msg/joint_encoders.hpp"
#include "obelisk_sensor_msgs/msg/true_sim_state.hpp"

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

        // TODO (@zolkin): Should this be public or protected?
        /**
         * @brief Creates a publisher, but first verifies if it is a Obelisk allowed
         *  message type.
         */
        template <typename MessageT, typename AllocatorT = std::allocator<void>>
        std::shared_ptr<
            rclcpp_lifecycle::LifecyclePublisher<MessageT, AllocatorT>>
        create_publisher(const std::string& topic_name, const rclcpp::QoS& qos,
            const rclcpp::PublisherOptionsWithAllocator<AllocatorT>& options =
                (rclcpp_lifecycle::create_default_publisher_options<
                    AllocatorT>())) {
            // Check if the message type is valid
            if (ValidMessage<MessageT, ObeliskMsgs>::value ||
                ValidMessage<MessageT, ROSAllowedMsgs>::value) {
                return rclcpp_lifecycle::LifecycleNode::create_publisher<
                    MessageT, AllocatorT>(topic_name, qos, options);
            }

            throw std::runtime_error(
                "Provided message type is not a valid Obelisk message!");
        };

        /**
         * @brief Creates a subscriber, but first verifies if it is a Obelisk allowed
         *  message type.
         */
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
            // Check if the message type is valid
            if (ValidMessage<MessageT, ObeliskMsgs>::value ||
                ValidMessage<MessageT, ROSAllowedMsgs>::value) {
                return rclcpp_lifecycle::LifecycleNode::create_subscription<
                    MessageT, CallbackT, AllocatorT, SubscriptionT,
                    MessageMemoryStrategyT>(topic_name, qos, std::move(callback), options,
                        msg_mem_strat);
            }

            throw std::runtime_error(
                "Provided message type is not a valid Obelisk message!");
        };

       private:

        // Allowed Obelisk message types
        using ObeliskMsgs    = std::tuple<
            obelisk_control_msgs::msg::PositionSetpoint,
            obelisk_estimator_msgs::msg::EstimatedState,
            obelisk_sensor_msgs::msg::JointEncoder,
            obelisk_sensor_msgs::msg::JointEncoders,
            obelisk_sensor_msgs::msg::TrueSimState
            >;
        // Allowed non-obelisk message types
        using ROSAllowedMsgs = std::tuple<rcl_interfaces::msg::ParameterEvent>;

        /**
         * Helper functions for determining if we have a valid message.
         * These functions recursively dissamble a tuple to check if the type
         * matches. See this stack overflow post:
         * https://stackoverflow.com/questions/25958259/how-do-i-find-out-if-a-tuple-contains-a-type
         */
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
