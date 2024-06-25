#pragma once
#include <any>
#include <chrono>
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
    explicit ObeliskNode(
        const std::string& node_name,
        const rclcpp::NodeOptions& options  = rclcpp::NodeOptions(),
        bool enable_communication_interface = true)
        : LifecycleNode(node_name, options, enable_communication_interface) {};

    ObeliskNode(const std::string& node_name, const std::string& namespace_,
                const rclcpp::NodeOptions& options  = rclcpp::NodeOptions(),
                bool enable_communication_interface = true)
        : LifecycleNode(node_name, namespace_, options,
                        enable_communication_interface) {};

    // TODO (@zolkin): Should this be public or protected?
    /**
     * @brief Creates a publisher, but first verifies if it is a Obelisk
     * allowed message type.
     *
     * @param topic_name the topic
     * @param qos
     * @param non_obelisk determines if we are allowed to publish non-obelisk
     * messages
     * @param options
     * @return the publisher
     */
    template <typename MessageT, typename AllocatorT = std::allocator<void>>
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<MessageT, AllocatorT>>
    create_publisher(
        const std::string& topic_name, const rclcpp::QoS& qos,
        bool non_obelisk = false,
        const rclcpp::PublisherOptionsWithAllocator<AllocatorT>& options =
            (rclcpp_lifecycle::create_default_publisher_options<
                AllocatorT>())) {
        // Check if the message type is valid
        if (!non_obelisk) {
            if (!(ValidMessage<MessageT, ObeliskMsgs>::value ||
                  ValidMessage<MessageT, ROSAllowedMsgs>::value)) {
                throw std::runtime_error(
                    "Provided message type is not a valid Obelisk message!");
            }
        } else {
            RCLCPP_WARN_STREAM(
                this->get_logger(),
                "Creating a publisher that can publish non-Obelisk messages. "
                "This may cause certain API incompatibilities.");
        }

        return rclcpp_lifecycle::LifecycleNode::create_publisher<MessageT,
                                                                 AllocatorT>(
            topic_name, qos, options);
    };

    /**
     * @brief Creates a subscriber, but first verifies if it is a Obelisk
     * allowed message type.
     *
     * @param topic_name the topic
     * @param qos
     * @param callback the callback function
     * @param non_obelisk determines if we are allowed to subscribe to
     * non-obelisk messages. Logs warning if true.
     * @param options
     * @return the subscription
     */
    template <
        typename MessageT, typename CallbackT,
        typename AllocatorT    = std::allocator<void>,
        typename SubscriptionT = rclcpp::Subscription<MessageT, AllocatorT>,
        typename MessageMemoryStrategyT =
            typename SubscriptionT::MessageMemoryStrategyType>
    std::shared_ptr<SubscriptionT> create_subscription(
        const std::string& topic_name, const rclcpp::QoS& qos,
        CallbackT&& callback, bool non_obelisk = false,
        const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>& options =
            rclcpp_lifecycle::create_default_subscription_options<AllocatorT>(),
        typename MessageMemoryStrategyT::SharedPtr msg_mem_strat =
            (MessageMemoryStrategyT::create_default())) {
        // Check if the message type is valid
        if (!non_obelisk) {
            if (!(ValidMessage<MessageT, ObeliskMsgs>::value ||
                  ValidMessage<MessageT, ROSAllowedMsgs>::value)) {
                throw std::runtime_error(
                    "Provided message type is not a valid Obelisk message!");
            }
        } else {
            RCLCPP_WARN_STREAM(
                this->get_logger(),
                "Creating a subscriber that can publish non-Obelisk messages. "
                "This may cause certain API incompatibilities.");
        }

        return rclcpp_lifecycle::LifecycleNode::create_subscription<
            MessageT, CallbackT, AllocatorT, SubscriptionT,
            MessageMemoryStrategyT>(topic_name, qos, std::move(callback),
                                    options, msg_mem_strat);
    };

  protected:
    /**
     * @brief Create a subscriber from a configuration string.
     *
     * @param config the configuration string
     * @param callback the callback function (generall of the form
     * std::bind(&MyFunc, this, _1))
     * @return the subscription
     */
    template <
        typename MessageT, typename CallbackT,
        typename AllocatorT    = std::allocator<void>,
        typename SubscriptionT = rclcpp::Subscription<MessageT, AllocatorT>>
    std::shared_ptr<SubscriptionT>
    CreateSubscriptionFromConfigStr(const std::string& config,
                                    CallbackT&& callback) {
        // Parse the configuration string
        const auto config_map   = ParseConfigStr(config);
        const std::string topic = GetTopic(config_map);
        const int depth         = GetHistoryDepth(config_map);
        const bool non_obelisk  = !GetIsObeliskMsg(config_map);

        // Create the subscriber
        return create_subscription<MessageT>(topic, depth, std::move(callback),
                                             non_obelisk);
    }

    /**
     * @brief Create a publisher from a configuration string
     *
     * @param config the configuration string
     * @return the publisher
     */
    template <typename MessageT, typename AllocatorT = std::allocator<void>>
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<MessageT, AllocatorT>>
    CreatePublisherFromConfigStr(const std::string& config) {
        // Parse the configuration string
        const auto config_map   = ParseConfigStr(config);
        const std::string topic = GetTopic(config_map);
        const int depth         = GetHistoryDepth(config_map);
        const bool non_obelisk  = !GetIsObeliskMsg(config_map);

        return create_publisher<MessageT>(topic, depth, non_obelisk);
    }

    /**
     * @brief Create a wall timer from a configuration string
     *
     * @param config the configuration string
     * @param callback the callback function
     */
    template <typename DurationT = std::milli, typename CallbackT>
    typename rclcpp::GenericTimer<CallbackT>::SharedPtr
    CreateWallTimerFromConfigStr(const std::string& config,
                                 CallbackT&& callback) {
        // Finest frequency is determined by DurationT
        using namespace std::chrono_literals;

        const auto config_map   = ParseConfigStr(config);
        const double period_sec = GetPeriod(config_map); // Period in seconds
        const auto period_dur   = std::chrono::duration<double, DurationT>(
            period_sec); // Convert period str to DurationT

        return this->create_wall_timer(
            period_dur, std::move(callback)); // TODO: Add call back group

        // TODO: Config the callback group
    }

    /**
     * @brief Parses the configuration string into a map from strings to
     * strings. the value strings are meant to be parsed in other functions.
     */
    std::map<std::string, std::string> ParseConfigStr(std::string config) {
        const std::string val_delim  = ":";
        const std::string type_delim = ",";

        std::map<std::string, std::string> config_map;

        // Parse the string
        size_t type_idx;
        while (!config.empty()) {
            type_idx = config.find(type_delim);
            if (type_idx == std::string::npos) {
                type_idx = config.length();
            }

            size_t val_idx = config.find(val_delim);
            if (val_idx == std::string::npos) {
                throw std::runtime_error(
                    "Invalid configuration string! Missing `:` before last "
                    "`,`.");
            }

            std::string config_id = config.substr(0, val_idx);

            std::string val =
                config.substr(val_idx + 1, type_idx - (val_idx + 1));
            config_map.emplace(config_id, val);
            config.erase(0, type_idx + type_delim.length());
        }

        return config_map;
    }

    /**
     * @brief Parses the configuration string map to see if there is a topic
     */
    std::string GetTopic(const std::map<std::string, std::string>& config_map) {
        std::string topic;
        for (auto& it : config_map) {
            if (it.first == "topic") {
                topic = it.second;
                break;
            }
        }

        if (topic.empty()) {
            throw std::runtime_error(
                "No topic name was provided in the configuration string.");
        }

        return topic;
    }

    /**
     * @brief Parses the configuration string map to see if there is a history
     * depth
     */
    int GetHistoryDepth(const std::map<std::string, std::string>& config_map) {
        int depth = DEFAULT_DEPTH;
        for (auto& it : config_map) {
            if (it.first == "history_depth") {
                depth = std::stoi(it.second);
                break;
            }
        }

        return depth;
    }

    /**
     * @brief Parses the configuration string map to see if this is restricted
     * to only obelisk messages
     */
    bool GetIsObeliskMsg(const std::map<std::string, std::string>& config_map) {
        bool obk_msg = DEFAULT_IS_OBK_MSG;
        for (auto& it : config_map) {
            if (it.first == "non_obelisk") {
                if (it.second == "true") {
                    obk_msg = false;
                }
                break;
            }
        }

        return obk_msg;
    }

    double GetPeriod(const std::map<std::string, std::string>& config_map) {
        double period = -1;
        for (auto& it : config_map) {
            if (it.first == "timer_period_sec") {
                period = std::stod(it.second);
                break;
            }
        }

        if (period <= 0) {
            throw std::runtime_error("Invalid timer period time provided");
        }

        return period;
    }

    std::string
    GetMessageName(const std::map<std::string, std::string>& config_map) {
        for (auto& it : config_map) {
            if (it.first == "message_type") {
                return it.second;
            }
        }

        throw std::runtime_error(
            "No message type provided in this config string!");
    }

    // --------- Member Variables --------- //
    std::vector<std::string> config_strs_;

  private:
    // --------- Member Variables --------- //
    static constexpr int DEFAULT_DEPTH       = 10;
    static constexpr bool DEFAULT_IS_OBK_MSG = true;

    // Allowed Obelisk message types
    using ObeliskMsgs = std::tuple<obelisk_control_msgs::msg::PositionSetpoint,
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
    template <typename T, typename Tuple> struct has_type;

    template <typename T> struct has_type<T, std::tuple<>> : std::false_type {};

    template <typename T, typename U, typename... Ts>
    struct has_type<T, std::tuple<U, Ts...>> : has_type<T, std::tuple<Ts...>> {
    };

    template <typename T, typename... Ts>
    struct has_type<T, std::tuple<T, Ts...>> : std::true_type {};

    template <typename T, typename Tuple>
    using ValidMessage = typename has_type<T, Tuple>::type;
};
} // namespace obelisk
