#pragma once
#include <any>
#include <chrono>
#include <tuple>

#include "obelisk_control_msgs/msg/position_setpoint.hpp"
#include "obelisk_estimator_msgs/msg/estimated_state.hpp"
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
// TODO: Make all settings -> setting if it not an array
namespace obelisk {
    class ObeliskNode : public rclcpp_lifecycle::LifecycleNode {
      public:
        explicit ObeliskNode(const std::string& node_name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
                             bool enable_communication_interface = true)
            : LifecycleNode(node_name, options, enable_communication_interface), CB_GROUP_NONE("None"),
              CB_GROUP_MUTUALLY_EXEC("MutuallyExclusiveCallbackGroup"), CB_GROUP_REENTRANT("ReentrantCallbackGroup") {
            this->declare_parameter<std::string>("callback_group_setting", "");

            RCLCPP_INFO_STREAM(this->get_logger(), node_name << " created.");
        };

        ObeliskNode(const std::string& node_name, const std::string& namespace_,
                    const rclcpp::NodeOptions& options  = rclcpp::NodeOptions(),
                    bool enable_communication_interface = true)
            : LifecycleNode(node_name, namespace_, options, enable_communication_interface), CB_GROUP_NONE("None"),
              CB_GROUP_MUTUALLY_EXEC("MutuallyExclusiveCallbackGroup"), CB_GROUP_REENTRANT("ReentrantCallbackGroup") {
            this->declare_parameter<std::string>("callback_group_setting", "");
        };

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
        create_publisher(const std::string& topic_name, const rclcpp::QoS& qos, bool non_obelisk = false,
                         const rclcpp::PublisherOptionsWithAllocator<AllocatorT>& options =
                             (rclcpp_lifecycle::create_default_publisher_options<AllocatorT>())) {
            // Check if the message type is valid
            if (!non_obelisk) {
                if (!(ValidMessage<MessageT, ObeliskMsgs>::value || ValidMessage<MessageT, ROSAllowedMsgs>::value)) {
                    throw std::runtime_error("Provided message type is not a valid Obelisk message!");
                }
            } else {
                RCLCPP_WARN_STREAM(this->get_logger(), "Creating a publisher that can publish non-Obelisk messages. "
                                                       "This may cause certain API incompatibilities.");
            }

            return rclcpp_lifecycle::LifecycleNode::create_publisher<MessageT, AllocatorT>(topic_name, qos, options);
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
        template <typename MessageT, typename CallbackT, typename AllocatorT = std::allocator<void>,
                  typename SubscriptionT          = rclcpp::Subscription<MessageT, AllocatorT>,
                  typename MessageMemoryStrategyT = typename SubscriptionT::MessageMemoryStrategyType>
        std::shared_ptr<SubscriptionT> create_subscription(
            const std::string& topic_name, const rclcpp::QoS& qos, CallbackT&& callback, bool non_obelisk = false,
            const rclcpp::SubscriptionOptionsWithAllocator<AllocatorT>& options =
                rclcpp_lifecycle::create_default_subscription_options<AllocatorT>(),
            typename MessageMemoryStrategyT::SharedPtr msg_mem_strat = (MessageMemoryStrategyT::create_default())) {
            // Check if the message type is valid
            if (!non_obelisk) {
                if (!(ValidMessage<MessageT, ObeliskMsgs>::value || ValidMessage<MessageT, ROSAllowedMsgs>::value)) {
                    throw std::runtime_error("Provided message type is not a valid Obelisk message!");
                }
            } else {
                RCLCPP_WARN_STREAM(this->get_logger(), "Creating a subscriber that can publish non-Obelisk messages. "
                                                       "This may cause certain API incompatibilities.");
            }

            return rclcpp_lifecycle::LifecycleNode::create_subscription<MessageT, CallbackT, AllocatorT, SubscriptionT,
                                                                        MessageMemoryStrategyT>(
                topic_name, qos, std::move(callback), options, msg_mem_strat);
        };

        /**
         * @brief Configures the node.
         *  Specifically, here we parse the the configuration string that determines
         * the callback groups and names.
         *
         * @return success if everything is executed.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn virtual on_configure(
            const rclcpp_lifecycle::State& prev_state) {
            rclcpp_lifecycle::LifecycleNode::on_configure(prev_state);

            // Parse the configuration groups for this node
            ParseCallbackGroupConfig(this->get_parameter("callback_group_setting").as_string());

            RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << " configured.");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief activates up the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn virtual on_activate(
            const rclcpp_lifecycle::State& prev_state) {
            rclcpp_lifecycle::LifecycleNode::on_activate(prev_state);

            RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << " activated.");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief deactivates up the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn virtual on_deactivate(
            const rclcpp_lifecycle::State& prev_state) {
            rclcpp_lifecycle::LifecycleNode::on_deactivate(prev_state);

            RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << " deactivated.");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief cleans up the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn virtual on_cleanup(
            const rclcpp_lifecycle::State& prev_state) {
            rclcpp_lifecycle::LifecycleNode::on_cleanup(prev_state);

            // Clear current data
            callback_groups_.clear();

            RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << " cleaned up.");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        // TODO: Make all the child classes call these methods in theirs

        /**
         * @brief shutsdown the node the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn virtual on_shutdown(
            const rclcpp_lifecycle::State& prev_state) {
            rclcpp_lifecycle::LifecycleNode::on_shutdown(prev_state);
            // Clear current data
            callback_groups_.clear();

            RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << " shutdown.");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

      protected:
        /**
         * @brief Create a subscriber from a configuration string.
         *
         * @param config the configuration string
         * @param callback the callback function (generall of the form
         * std::bind(&MyFunc, this, _1))
         * @return the subscription
         */
        template <typename MessageT, typename CallbackT, typename AllocatorT = std::allocator<void>,
                  typename SubscriptionT = rclcpp::Subscription<MessageT, AllocatorT>>
        std::shared_ptr<SubscriptionT> CreateSubscriptionFromConfigStr(const std::string& config,
                                                                       CallbackT&& callback) {
            // Parse the configuration string
            const auto config_map                = ParseConfigStr(config);
            const std::string topic              = GetTopic(config_map);
            const int depth                      = GetHistoryDepth(config_map);
            const bool non_obelisk               = !GetIsObeliskMsg(config_map);

            rclcpp::CallbackGroup::SharedPtr cbg = nullptr; // default group
            try {
                // Get the callback group based on the string name
                cbg = callback_groups_.at(GetCallbackGroupName(config_map));
            } catch (const std::exception& e) {
            }

            rclcpp::SubscriptionOptions options;
            options.callback_group = cbg;

            // Create the subscriber
            return create_subscription<MessageT>(topic, depth, std::move(callback), non_obelisk, options);
        }

        /**
         * @brief Create a publisher from a configuration string.
         *
         * @param config the configuration string.
         * @return the publisher.
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
         * @brief Create a wall timer from a configuration string.
         *
         * @param config the configuration string.
         * @param callback the callback function.
         *
         * @return the timer.
         */
        template <typename CallbackT>
        typename rclcpp::GenericTimer<CallbackT>::SharedPtr CreateWallTimerFromConfigStr(const std::string& config,
                                                                                         CallbackT&& callback) {
            // Finest frequency is determined by DurationT
            using namespace std::chrono_literals;

            const auto config_map                = ParseConfigStr(config);
            const double period_sec              = GetPeriod(config_map); // Period in seconds

            rclcpp::CallbackGroup::SharedPtr cbg = nullptr;               // default group
            try {
                // Get the callback group based on the string name
                cbg = callback_groups_.at(GetCallbackGroupName(config_map));
            } catch (const std::exception& e) {
            }

            // Create the timer
            return this->create_wall_timer(std::chrono::microseconds(static_cast<uint>(1e6 * period_sec)),
                                           std::move(callback), cbg);
        }

        /**
         * @brief Parses the configuration string into a map from strings to
         * strings. the value strings are meant to be parsed in other functions.
         *
         * @return a map of configuration options to their settings as strings.
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
                    throw std::runtime_error("Invalid configuration string! Missing `:` before last "
                                             "`,`.");
                }

                std::string config_id = config.substr(0, val_idx);

                std::string val       = config.substr(val_idx + 1, type_idx - (val_idx + 1));
                config_map.emplace(config_id, val);
                config.erase(0, type_idx + type_delim.length());
            }

            return config_map;
        }

        /**
         * @brief Parses the configuration string map to see if there is a topic.
         *  Throws an error if there is no topic.
         *
         * @param config_map the map created by ParseConfigStr.
         * @return the topic.
         */
        std::string GetTopic(const std::map<std::string, std::string>& config_map) {
            try {
                std::string topic = config_map.at("topic");
                return topic;
            } catch (const std::exception& e) {
                throw std::runtime_error("No topic name was provided in the configuration string.");
            }
        }

        /**
         * @brief Parses the configuration string map to see if there is a history
         * depth.
         *
         * @param config_map the map created by ParseConfigStr.
         * @return the message history depth.
         */
        int GetHistoryDepth(const std::map<std::string, std::string>& config_map) {
            try {
                return std::stoi(config_map.at("history_depth"));
            } catch (const std::exception& e) {
                return DEFAULT_DEPTH;
            }
        }

        /**
         * @brief Parses the configuration string map to see if this is restricted
         * to only obelisk messages.
         *
         * @param config_map the map created by ParseConfigStr.
         * @return use obelisk messages or not.
         */
        bool GetIsObeliskMsg(const std::map<std::string, std::string>& config_map) {
            try {
                if (config_map.at("non_obelisk") == "true") {
                    return false;
                } else {
                    return true;
                }
            } catch (const std::exception& e) {
                return DEFAULT_IS_OBK_MSG;
            }
        }

        /**
         * @brief Parse the configuration string map to get the period of the timer.
         *  Throws an error if there is no period.
         *
         * @param config_map the map created by ParseConfigStr.
         * @return the period (in seconds).
         */
        double GetPeriod(const std::map<std::string, std::string>& config_map) {
            try {
                double period = std::stod(config_map.at("timer_period_sec"));
                if (period <= 0) {
                    throw std::runtime_error("Period must be >= 0!");
                }

                return period;
            } catch (const std::exception& e) {
                throw std::runtime_error("Invalid timer period time provided");
            }
        }

        /**
         * @brief Parses the configuration string map to get the message name from a
         * config string. Throws an error if there is no message name.
         *
         * @param config_map the map created by ParseConfigStr.
         * @return the message name.
         */
        std::string GetMessageName(const std::map<std::string, std::string>& config_map) {
            try {
                return config_map.at("message_type");
            } catch (const std::exception& e) {
                throw std::runtime_error("No message type provided in this config string!");
            }
        }

        /**
         * @brief Parses the configuration string map to get the callback group
         * name.
         *
         * @param config_map the map created by ParseConfigStr.
         * @return the callback group name.
         */
        std::string GetCallbackGroupName(const std::map<std::string, std::string>& config_map) {
            try {
                return config_map.at("callback_group");
            } catch (const std::exception& e) {
                return CB_GROUP_NONE;
            }
        }

        /**
         * @brief Parses a configuration string to determine the names and types of
         * callback groups. Sets callback_groups_.
         *
         * @param config the configuration string.
         */
        void ParseCallbackGroupConfig(const std::string& config) {
            // Parse the config string into group name, group type
            const auto callback_group_names = ParseConfigStr(config);

            callback_groups_.clear();

            // Iterate through all the group names
            for (const auto& cbg : callback_group_names) {
                // If the name is associated with a specific type, then make that
                // type If the name is not then assign to the default callback group
                if (cbg.second == CB_GROUP_MUTUALLY_EXEC) {
                    callback_groups_.emplace(cbg.first,
                                             this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
                } else if (cbg.second == CB_GROUP_REENTRANT) {
                    callback_groups_.emplace(cbg.first,
                                             this->create_callback_group(rclcpp::CallbackGroupType::Reentrant));
                } else {
                    // Node's default callback group
                    callback_groups_.emplace(cbg.first, nullptr);
                }
            }

            callback_groups_.emplace(CB_GROUP_NONE, nullptr);
        }

        // --------- Member Variables --------- //
        const std::string CB_GROUP_NONE;
        const std::string CB_GROUP_MUTUALLY_EXEC;
        const std::string CB_GROUP_REENTRANT;

        // Map callback group names to their callbacks
        std::map<std::string, rclcpp::CallbackGroup::SharedPtr> callback_groups_;

      private:
        // --------- Member Variables --------- //
        static constexpr int DEFAULT_DEPTH       = 10;
        static constexpr bool DEFAULT_IS_OBK_MSG = true;

        // Allowed Obelisk message types
        using ObeliskMsgs =
            std::tuple<obelisk_control_msgs::msg::PositionSetpoint, obelisk_estimator_msgs::msg::EstimatedState,
                       obelisk_sensor_msgs::msg::JointEncoders, obelisk_sensor_msgs::msg::TrueSimState>;
        // Allowed non-obelisk message types
        using ROSAllowedMsgs = std::tuple<rcl_interfaces::msg::ParameterEvent>;

        /**
         * Helper functions for determining if we have a valid message.
         * These functions recursively disassemble a tuple to check if the type
         * matches. See this stackoverflow post:
         * https://stackoverflow.com/questions/25958259/how-do-i-find-out-if-a-tuple-contains-a-type
         */
        template <typename T, typename Tuple> struct has_type;

        template <typename T> struct has_type<T, std::tuple<>> : std::false_type {};

        template <typename T, typename U, typename... Ts>
        struct has_type<T, std::tuple<U, Ts...>> : has_type<T, std::tuple<Ts...>> {};

        template <typename T, typename... Ts> struct has_type<T, std::tuple<T, Ts...>> : std::true_type {};

        template <typename T, typename Tuple> using ValidMessage = typename has_type<T, Tuple>::type;
    };
} // namespace obelisk