#pragma once
#include <chrono>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <yaml-cpp/yaml.h>

#include "obelisk_control_msgs/msg/pd_feed_forward.hpp"
#include "obelisk_control_msgs/msg/position_setpoint.hpp"

#include "obelisk_estimator_msgs/msg/estimated_state.hpp"

#include "obelisk_sensor_msgs/msg/obk_frame_pose.hpp"
#include "obelisk_sensor_msgs/msg/obk_image.hpp"
#include "obelisk_sensor_msgs/msg/obk_joint_encoders.hpp"
#include "obelisk_sensor_msgs/msg/true_sim_state.hpp"

#include "obelisk_std_msgs/msg/float_multi_array.hpp"
#include "obelisk_std_msgs/msg/u_int8_multi_array.hpp"

namespace obelisk {
    namespace internal {
        // ------------------------------------ //
        // ------------ Publishers ------------ //
        // ------------------------------------ //

        /**
         * @brief Internal class used to create a heirarchy to allow polymorphism over publisher types.
         */
        class ObeliskPublisherBase {
          public:
            virtual void Activate()   = 0;
            virtual void Deactivate() = 0;
            virtual void Release()    = 0;
        };

        template <typename MessageT> class ObeliskPublisher : public ObeliskPublisherBase {
          public:
            ObeliskPublisher(typename rclcpp_lifecycle::LifecyclePublisher<MessageT>::SharedPtr publisher)
                : publisher_(publisher) {}

            void Activate() override { publisher_->on_activate(); }
            void Deactivate() override { publisher_->on_deactivate(); }
            void Release() override { publisher_.reset(); }

            typename rclcpp_lifecycle::LifecyclePublisher<MessageT>::SharedPtr GetPublisher() { return publisher_; }

          private:
            typename rclcpp_lifecycle::LifecyclePublisher<MessageT>::SharedPtr publisher_;
        };

        struct ObeliskPublisherInfo {
            std::function<std::shared_ptr<ObeliskPublisherBase>(const YAML::Node& entry)> creator;
        };

        // ------------------------------------- //
        // ------------ Subscribers ------------ //
        // ------------------------------------- //
        class ObeliskSubscriptionBase {
          public:
            virtual void Release() = 0;
        };

        template <typename MessageT> class ObeliskSubscription : public ObeliskSubscriptionBase {
          public:
            ObeliskSubscription(typename rclcpp::Subscription<MessageT>::SharedPtr subscription)
                : subscription_(subscription) {}

            void Release() override { subscription_.reset(); }

            typename rclcpp::Subscription<MessageT>::SharedPtr GetSubscription() { return subscription_; }

          private:
            typename rclcpp::Subscription<MessageT>::SharedPtr subscription_;
        };

        struct ObeliskSubscriptionInfo {
            std::function<std::shared_ptr<ObeliskSubscriptionBase>(const YAML::Node& entry)> creator;
        };

        // ------------------------------------- //
        // --------------- Timers -------------- //
        // ------------------------------------- //
        struct ObeliskTimerInfo {
            std::function<rclcpp::TimerBase::SharedPtr(const YAML::Node& entry)> creator;
        };

    } // namespace internal

    /**
     * @brief Abstract generic Obelisk node.
     *
     * A lifecycle node whose publishers, subscribers, and timers are configured from a single
     * ROS string parameter named ``obelisk_settings`` whose value is a YAML document. The launch-side
     * producer (``obelisk_py.core.utils.launch_utils``) builds that YAML directly from the launch
     * config, so no flat ``field:value,...`` config-string format is needed.
     *
     * Components are registered by ``key`` via ``RegisterObkPublisher`` / ``RegisterObkSubscription``
     * / ``RegisterObkTimer``. At ``on_configure`` time the node walks the parsed YAML and instantiates
     * each registered component from the entry whose ``key`` field matches.
     *
     * Lifecycle Callbacks
     * -------------------
     * The on_configure callback should do the following:
     *   * Instantiate required ROS parameters.
     *   * Instantiate optional ROS parameters.
     *   * Declare publishers, timers, and subscribers (timers start deactivated).
     *   * Initialize stateful quantities.
     *
     * The on_activate callback should do the following:
     *   * Activate any timers that were declared in on_configure.
     *
     * The on_deactivate callback should do the following:
     *   * Deactivate any timers that were activated in on_activate.
     */
    class ObeliskNode : public rclcpp_lifecycle::LifecycleNode {
      public:
        explicit ObeliskNode(const std::string& node_name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions(),
                             bool enable_communication_interface = true)
            : LifecycleNode(node_name, options, enable_communication_interface), CB_GROUP_NONE("None"),
              CB_GROUP_MUTUALLY_EXEC("MutuallyExclusiveCallbackGroup"), CB_GROUP_REENTRANT("ReentrantCallbackGroup") {
            this->declare_parameter<std::string>(OBK_SETTINGS_PARAM, "");
            this->declare_parameter<std::string>("params_path", "");
            RCLCPP_INFO_STREAM(this->get_logger(), node_name << " created.");
        };

        ObeliskNode(const std::string& node_name, const std::string& namespace_,
                    const rclcpp::NodeOptions& options  = rclcpp::NodeOptions(),
                    bool enable_communication_interface = true)
            : LifecycleNode(node_name, namespace_, options, enable_communication_interface), CB_GROUP_NONE("None"),
              CB_GROUP_MUTUALLY_EXEC("MutuallyExclusiveCallbackGroup"), CB_GROUP_REENTRANT("ReentrantCallbackGroup") {
            this->declare_parameter<std::string>(OBK_SETTINGS_PARAM, "");
            this->declare_parameter<std::string>("params_path", "");
            RCLCPP_INFO_STREAM(this->get_logger(), node_name << " created.");
        };

        // ---------------------------------------------------------- //
        // -------------------- Component Register ------------------ //
        // ---------------------------------------------------------- //

        /**
         * @brief Register a publisher under ``key``.
         *
         * The publisher is created at on_configure time from the entry in obelisk_settings.publishers
         * whose ``key`` field equals ``key``.
         */
        template <typename MessageT> void RegisterObkPublisher(const std::string& key) {
            internal::ObeliskPublisherInfo info;
            info.creator = [this](const YAML::Node& entry) {
                auto publisher = this->CreatePublisherFromEntry<MessageT>(entry);
                return std::make_shared<internal::ObeliskPublisher<MessageT>>(publisher);
            };
            registered_publishers_[key] = info;
        }

        template <typename MessageT>
        typename rclcpp_lifecycle::LifecyclePublisher<MessageT>::SharedPtr GetPublisher(const std::string& key) {
            auto it = publishers_.find(key);
            if (it != publishers_.end()) {
                auto pub = std::dynamic_pointer_cast<internal::ObeliskPublisher<MessageT>>(it->second);
                if (pub) {
                    return pub->GetPublisher();
                }
                throw std::runtime_error("Publisher is empty!");
            }
            throw std::runtime_error("Publisher not found for the key!");
        }

        /**
         * @brief Register a subscription under ``key``.
         */
        template <typename MessageT, typename CallbackT>
        void RegisterObkSubscription(const std::string& key, CallbackT&& callback) {
            internal::ObeliskSubscriptionInfo info;
            info.creator = [this, callback](const YAML::Node& entry) {
                auto subscription = this->CreateSubscriptionFromEntry<MessageT>(entry, std::move(callback));
                return std::make_shared<internal::ObeliskSubscription<MessageT>>(subscription);
            };
            registered_subscriptions_[key] = info;
        }

        template <typename MessageT>
        typename rclcpp::Subscription<MessageT>::SharedPtr GetSubscription(const std::string& key) {
            auto it = subscriptions_.find(key);
            if (it != subscriptions_.end()) {
                auto sub = std::dynamic_pointer_cast<internal::ObeliskSubscription<MessageT>>(it->second);
                if (sub) {
                    return sub->GetSubscription();
                }
                throw std::runtime_error("Subscription is empty!");
            }
            throw std::runtime_error("Subscription not found for the key!");
        }

        /**
         * @brief Register a timer under ``key``.
         */
        template <typename CallbackT> void RegisterObkTimer(const std::string& key, CallbackT&& callback) {
            internal::ObeliskTimerInfo info;
            info.creator = [this, callback](const YAML::Node& entry) {
                return this->CreateWallTimerFromEntry(entry, std::move(callback));
            };
            registered_timers_[key] = info;
        }

        typename rclcpp::TimerBase::SharedPtr GetTimer(const std::string& key) {
            auto it = timers_.find(key);
            if (it != timers_.end()) {
                return it->second;
            }
            throw std::runtime_error("Timer not found for the key!");
        }

        // ---------------------------------------------------------- //
        // -------------------- Lifecycle Hooks --------------------- //
        // ---------------------------------------------------------- //

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn virtual on_configure(
            const rclcpp_lifecycle::State& prev_state) {
            rclcpp_lifecycle::LifecycleNode::on_configure(prev_state);

            obk_settings_     = LoadObkSettings();
            publisher_specs_  = ToVector(obk_settings_["publishers"]);
            subscriber_specs_ = ToVector(obk_settings_["subscribers"]);
            timer_specs_      = ToVector(obk_settings_["timers"]);

            BuildCallbackGroups(obk_settings_["callback_groups"]);
            CreatePublishers();
            CreateSubscriptions();
            CreateTimers();

            RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << " configured.");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn virtual on_activate(
            const rclcpp_lifecycle::State& prev_state) {
            rclcpp_lifecycle::LifecycleNode::on_activate(prev_state);

            for (auto& [key, pub] : publishers_) {
                if (pub) {
                    pub->Activate();
                }
            }
            for (auto& [key, timer] : timers_) {
                if (timer) {
                    timer->reset();
                }
            }
            RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << " activated.");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn virtual on_deactivate(
            const rclcpp_lifecycle::State& prev_state) {
            rclcpp_lifecycle::LifecycleNode::on_deactivate(prev_state);

            for (auto& [key, pub] : publishers_) {
                if (pub) {
                    pub->Deactivate();
                }
            }
            for (auto& [key, timer] : timers_) {
                if (timer) {
                    timer->cancel();
                }
            }
            RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << " deactivated.");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn virtual on_cleanup(
            const rclcpp_lifecycle::State& prev_state) {
            rclcpp_lifecycle::LifecycleNode::on_cleanup(prev_state);
            ReleaseAll();
            RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << " cleaned up.");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn virtual on_shutdown(
            const rclcpp_lifecycle::State& prev_state) {
            rclcpp_lifecycle::LifecycleNode::on_shutdown(prev_state);
            ReleaseAll();
            RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << " shutdown.");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

      protected:
        // The single ROS parameter that carries all Obelisk-internal settings as a YAML string.
        static constexpr const char* OBK_SETTINGS_PARAM = "obelisk_settings";

        /**
         * @brief Loads the obelisk_settings ROS parameter and returns it as a YAML::Node.
         *
         * Returns an empty (null) YAML::Node if the parameter is empty.
         */
        YAML::Node LoadObkSettings() {
            const std::string raw = this->get_parameter(OBK_SETTINGS_PARAM).as_string();
            if (raw.empty()) {
                return YAML::Node{};
            }
            try {
                return YAML::Load(raw);
            } catch (const YAML::Exception& e) {
                throw std::runtime_error(std::string("Failed to parse obelisk_settings YAML: ") + e.what());
            }
        }

        /**
         * @brief Convert a YAML sequence to a std::vector<YAML::Node> (skipping null/missing).
         */
        static std::vector<YAML::Node> ToVector(const YAML::Node& node) {
            std::vector<YAML::Node> out;
            if (node && node.IsSequence()) {
                out.reserve(node.size());
                for (const auto& item : node) {
                    out.push_back(item);
                }
            }
            return out;
        }

        /**
         * @brief Find an entry in ``entries`` whose ``key`` field equals ``key``. Returns null Node if absent.
         */
        static YAML::Node FindEntryByKey(const std::vector<YAML::Node>& entries, const std::string& key) {
            for (const auto& entry : entries) {
                if (entry["key"] && entry["key"].as<std::string>() == key) {
                    return entry;
                }
            }
            return YAML::Node{};
        }

        /**
         * @brief Build callback groups from a YAML::Node map of {name: type_name}.
         */
        void BuildCallbackGroups(const YAML::Node& spec) {
            callback_groups_.clear();
            if (spec && spec.IsMap()) {
                for (const auto& kv : spec) {
                    const std::string name      = kv.first.as<std::string>();
                    const std::string type_name = kv.second.as<std::string>();
                    if (type_name == CB_GROUP_MUTUALLY_EXEC) {
                        callback_groups_.emplace(
                            name, this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
                    } else if (type_name == CB_GROUP_REENTRANT) {
                        callback_groups_.emplace(name,
                                                 this->create_callback_group(rclcpp::CallbackGroupType::Reentrant));
                    } else {
                        callback_groups_.emplace(name, nullptr);
                        RCLCPP_WARN_STREAM(this->get_logger(),
                                           "Unknown callback group type '" << type_name << "' for group '" << name
                                                                           << "'; using default.");
                    }
                }
            }
            callback_groups_.emplace(CB_GROUP_NONE, nullptr);
        }

        /**
         * @brief Look up a callback group by the (string) value in entry["callback_group"]. Defaults to nullptr.
         */
        rclcpp::CallbackGroup::SharedPtr ResolveCallbackGroup(const YAML::Node& entry) {
            if (!entry || !entry["callback_group"]) {
                return nullptr;
            }
            const std::string name = entry["callback_group"].as<std::string>();
            auto it                = callback_groups_.find(name);
            if (it != callback_groups_.end()) {
                return it->second;
            }
            return nullptr;
        }

        /**
         * @brief Common helper: read entry["topic"] (required) and entry["history_depth"] (optional, default 10).
         */
        static std::string GetTopic(const YAML::Node& entry) {
            if (!entry["topic"]) {
                throw std::runtime_error("Entry is missing required field 'topic'.");
            }
            return entry["topic"].as<std::string>();
        }

        static int GetHistoryDepth(const YAML::Node& entry) {
            if (entry["history_depth"]) {
                return entry["history_depth"].as<int>();
            }
            return DEFAULT_DEPTH;
        }

        static double GetTimerPeriodSec(const YAML::Node& entry) {
            if (!entry["timer_period_sec"]) {
                throw std::runtime_error("Timer entry is missing required field 'timer_period_sec'.");
            }
            const double period = entry["timer_period_sec"].as<double>();
            if (period <= 0) {
                throw std::runtime_error("timer_period_sec must be > 0.");
            }
            return period;
        }

        // -------------------- Component creation --------------------

        void CreatePublishers() {
            for (const auto& [key, info] : registered_publishers_) {
                const auto entry = FindEntryByKey(publisher_specs_, key);
                if (!entry) {
                    RCLCPP_WARN_STREAM(this->get_logger(),
                                       "Publisher <" << key << "> has no entry in obelisk_settings.publishers.");
                    continue;
                }
                publishers_[key] = info.creator(entry);
            }
        }

        void CreateSubscriptions() {
            for (const auto& [key, info] : registered_subscriptions_) {
                const auto entry = FindEntryByKey(subscriber_specs_, key);
                if (!entry) {
                    RCLCPP_WARN_STREAM(this->get_logger(),
                                       "Subscription <" << key << "> has no entry in obelisk_settings.subscribers.");
                    continue;
                }
                subscriptions_[key] = info.creator(entry);
            }
        }

        void CreateTimers() {
            for (const auto& [key, info] : registered_timers_) {
                const auto entry = FindEntryByKey(timer_specs_, key);
                if (!entry) {
                    RCLCPP_WARN_STREAM(this->get_logger(),
                                       "Timer <" << key << "> has no entry in obelisk_settings.timers.");
                    continue;
                }
                timers_[key] = info.creator(entry);
                timers_[key]->cancel(); // Pause until on_activate
            }
        }

        template <typename MessageT, typename CallbackT, typename AllocatorT = std::allocator<void>,
                  typename SubscriptionT = rclcpp::Subscription<MessageT, AllocatorT>>
        std::shared_ptr<SubscriptionT> CreateSubscriptionFromEntry(const YAML::Node& entry, CallbackT&& callback) {
            const std::string topic = GetTopic(entry);
            const int depth         = GetHistoryDepth(entry);
            rclcpp::SubscriptionOptions options;
            options.callback_group = ResolveCallbackGroup(entry);
            return create_subscription<MessageT>(topic, depth, std::move(callback), options);
        }

        template <typename MessageT, typename AllocatorT = std::allocator<void>>
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<MessageT, AllocatorT>>
        CreatePublisherFromEntry(const YAML::Node& entry) {
            const std::string topic = GetTopic(entry);
            const int depth         = GetHistoryDepth(entry);
            return create_publisher<MessageT>(topic, depth);
        }

        template <typename CallbackT>
        typename rclcpp::TimerBase::SharedPtr CreateWallTimerFromEntry(const YAML::Node& entry, CallbackT&& callback) {
            const double period_sec              = GetTimerPeriodSec(entry);
            rclcpp::CallbackGroup::SharedPtr cbg = ResolveCallbackGroup(entry);
            return this->create_wall_timer(std::chrono::microseconds(static_cast<uint>(1e6 * period_sec)),
                                           std::move(callback), cbg);
        }

        // -------------------- Cleanup helper --------------------

        void ReleaseAll() {
            callback_groups_.clear();

            for (auto& [key, pub] : publishers_) {
                if (pub) {
                    pub->Release();
                }
            }
            publishers_.clear();
            registered_publishers_.clear();

            for (auto& [key, sub] : subscriptions_) {
                if (sub) {
                    sub->Release();
                }
            }
            subscriptions_.clear();
            registered_subscriptions_.clear();

            for (auto& [key, timer] : timers_) {
                if (timer) {
                    timer->cancel();
                    timer.reset();
                }
            }
            timers_.clear();
            registered_timers_.clear();

            publisher_specs_.clear();
            subscriber_specs_.clear();
            timer_specs_.clear();
        }

        // ------------ Member variables ------------ //
        const std::string CB_GROUP_NONE;
        const std::string CB_GROUP_MUTUALLY_EXEC;
        const std::string CB_GROUP_REENTRANT;

        // Parsed from `obelisk_settings` at on_configure time.
        YAML::Node obk_settings_;
        std::vector<YAML::Node> publisher_specs_;
        std::vector<YAML::Node> subscriber_specs_;
        std::vector<YAML::Node> timer_specs_;

        // Callback groups by name.
        std::map<std::string, rclcpp::CallbackGroup::SharedPtr> callback_groups_;

        // Registered components (factory closures keyed by string).
        std::unordered_map<std::string, internal::ObeliskPublisherInfo> registered_publishers_;
        std::unordered_map<std::string, std::shared_ptr<internal::ObeliskPublisherBase>> publishers_;

        std::unordered_map<std::string, internal::ObeliskSubscriptionInfo> registered_subscriptions_;
        std::unordered_map<std::string, std::shared_ptr<internal::ObeliskSubscriptionBase>> subscriptions_;

        std::unordered_map<std::string, internal::ObeliskTimerInfo> registered_timers_;
        std::unordered_map<std::string, rclcpp::TimerBase::SharedPtr> timers_;

      private:
        static constexpr int DEFAULT_DEPTH = 10;
    };
} // namespace obelisk
