#pragma once
#include <any>
#include <tuple>

#include "obelisk_control_msgs/msg/position_setpoint.hpp"
#include "obelisk_estimator_msgs/msg/estimated_state.hpp"
#include "obelisk_sensor_msgs/msg/joint_encoder.hpp"
#include "obelisk_sensor_msgs/msg/joint_encoders.hpp"
#include "obelisk_sensor_msgs/msg/true_sim_state.hpp"
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
                  node_name, options, enable_communication_interface) {
                  // id_funcs.emplace_back("test1", &ObeliskNode::TestCallback);
              };

        ObeliskNode(const std::string& node_name, const std::string& namespace_,
            const rclcpp::NodeOptions& options  = rclcpp::NodeOptions(),
            bool enable_communication_interface = true)
            : LifecycleNode(node_name, namespace_, options,
                  enable_communication_interface) {};

        // TODO (@zolkin): Should this be public or protected?
        /**
         * @brief Creates a publisher, but first verifies if it is a Obelisk
         * allowed message type.
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
         * @brief Creates a subscriber, but first verifies if it is a Obelisk
         * allowed message type.
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
                    MessageMemoryStrategyT>(topic_name, qos,
                    std::move(callback), options, msg_mem_strat);
            }

            throw std::runtime_error(
                "Provided message type is not a valid Obelisk message!");
        };

        // The subscription type depends on the message type, which means that
        // the return type of this function depends on the message type
        // template<typename MessageT,
        //     typename AllocatorT = std::allocator<void>,
        //     typename SubscriptionT = rclcpp::Subscription<MessageT,
        //     AllocatorT>>
        std::shared_ptr<std::any> CreateSubscriptionFromConfigStr(
            const std::string& config) {
            auto sub = create_subscription<
                obelisk_control_msgs::msg::PositionSetpoint>("topic", 10,
                std::bind(
                    &ObeliskNode::TestCallback, this, std::placeholders::_1));
            return sub;
            // id_funcs.at(0).second()
        }

       protected:
        // If GetCallback is templated then multiple functions are compiled
        //  so in the above code it will need to know which function should be
        //  called, therefore, there will need to be some type of if/switch
        //  statement
        // template<typename CallbackT>
        // CallbackT GetCallback(const std::string& cb_identifier) {
        //     if (cb_identifier == "test1") {
        //         return
        //         std::bind(&ObeliskNode::TestCallback<rcl_interfaces::msg::ParameterEvent>,
        //             this, std::placeholders::_1);
        //     }

        //     throw std::runtime_error(
        //         "Bad callback identifier");
        // }

        // template<typename MessageT>
        void TestCallback(
            const obelisk_control_msgs::msg::PositionSetpoint& msg) {
            std::cout << "Test callback" << std::endl;
        }

        // typedef void (ObeliskNode::*FunctionPointer)();
        // std::vector<std::pair<std::string, std::function<void(std::any)>>>
        // id_funcs;

       private:
        // Allowed Obelisk message types
        using ObeliskMsgs =
            std::tuple<obelisk_control_msgs::msg::PositionSetpoint,
                obelisk_estimator_msgs::msg::EstimatedState,
                obelisk_sensor_msgs::msg::JointEncoder,
                obelisk_sensor_msgs::msg::JointEncoders,
                obelisk_sensor_msgs::msg::TrueSimState>;
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

// ChatGPT class
// class TestClass {
//     TestClass() {
//         // add a function pointer to the vector
//         id_funcs.emplace_back("test1", &ObeliskNode::TestCallback);
//     }

//     // Create an object based on the string
//     // Templated return type
//     template<typename MessageT,
//     typename AllocatorT = std::allocator<void>,
//     typename SubscriptionT = rclcpp::Subscription<MessageT, AllocatorT>>
//     std::shared_ptr<SubscriptionT> CreateSubscriptionFromConfigStr(
//         const std::string& config) {
//         create_subscription<obelisk_control_msgs::msg::PositionSetpoint>(
//             "topic", 10, std::bind(id_funcs.at(0).second(), this,
//             std::placeholders::_1));
//     }
// }
