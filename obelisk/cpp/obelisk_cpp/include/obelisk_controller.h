#include "obelisk_node.h"
// #include "rclcpp_lifecycle/lifecycle_node_interface.hpp"

namespace obelisk {
template <typename... MessagesT>
// TODO: Can make two of these with different names, but do pretty much the same
// thing (one for pub, one for sub)
struct MessagePack {
    using MessagePackType = std::tuple<MessagesT...>;
    // using PublisherPack = std::tuple<>
    using PublisherPackType =
        std::tuple<typename rclcpp::Publisher<MessagesT>::SharedPtr...>;
    using SubscriberPackType =
        std::tuple<typename rclcpp::Subscription<MessagesT>::SharedPtr...>;
    using SubscriberCallbackPackType =
        std::tuple<std::function<void(MessagesT)>...>;
    using TimerCallbackPackType = std::tuple<std::function<MessagesT(void)>...>;
};

// TODO: Consider multiple types here (variadic)
template <typename RequiredControlMessageT, typename RequiredEstimatorMessagesT,
          typename ControlMessagesT, typename EstimatorMessagesT>
class ObeliskController : public ObeliskNode {
  public:
    explicit ObeliskController(const std::string& name)
        : ObeliskNode(name, pub_config_param_names, sub_config_param_names,
                      timer_config_param_names, cb_config_param_names) {
        // Declare all paramters
        this->declare_parameter<std::string>("callback_group_config_strs");
        this->declare_parameter<std::string>("timer_ctrl_config_str");
        this->declare_parameter<std::string>("pub_ctrl_config_str");
        this->declare_parameter<std::string>("sub_est_config_str");
        // "topic:topic1,message_type:PositionSetpoint"
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State& prev_state) {
        // Get the config strings that we know about
        config_strs_.emplace_back(
            this->get_parameter("pub_ctrl_config_str").as_string());
        config_strs_.emplace_back(
            this->get_parameter("sub_est_config_str").as_string());
        config_strs_.emplace_back(
            this->get_parameter("timer_ctrl_config_str").as_string());
        config_strs_.emplace_back(
            this->get_parameter("callback_group_config_strs").as_string());

        CreateAllControllerPublishers();

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
            CallbackReturn::SUCCESS;
    }

    // template <typename CallbackT>
    // void CreateStateEstimatorSubscriber(CallbackT&& callback) {
    //     // Create the state estimator subscriber with the proper
    //     configuration state_estimator_subscriber_ =
    //         CreateSubscriptionFromConfigStr<StateMessageT>(
    //             this->get_parameter("sub_est_config_str"),
    //             std::move(callback));
    // };

    // // I can do this one recursively with a variadic template (i.e. recurse
    // down
    // // the stack of types and call this on the proper publisher tuple)
    void CreateAllControllerPublishers() {
        // TODO: Verify this assigns correctly
        std::apply([this](auto&... ts) { (..., CreatePubWithStr(ts)); },
                   ctrl_messages_);
    };

    // // TODO: What does the timer take?
    // template <typename CallbackT>
    // typename rclcpp::GenericTimer<CallbackT>::SharedPtr
    // CreateControlTimer(CallbackT&& callback) {
    //     // Create the state estimator subscriber with the proper
    //     configuration return CreateWallTimerFromConfigStr(
    //         this->get_parameter("timer_ctrl_config_str"),
    //         std::move(callback));
    // };

  protected:
    // Hold a tuple of publishers and tuple of subscribers, defined from a
    // variadic template
    using ControlMessagesTuple   = typename ControlMessagesT::MessagePackType;
    using EstimatorMessagesTuple = typename EstimatorMessagesT::MessagePackType;
    using PublisherTuple         = typename ControlMessagesT::PublisherPackType;
    using SubscriberTuple = typename EstimatorMessagesT::SubscriberPackType;

    template <typename MessageT>
    using PublisherType = typename rclcpp::Publisher<MessageT>::SharedPtr;

    template <typename MessageT>
    using SubscriberType = typename rclcpp::Subscription<MessageT>::SharedPtr;

    // List of all the publishers
    PublisherTuple control_publishers_;

    // List of all the subscribers
    SubscriberTuple estimator_subscribers_;

    // Tuple used just for the types
    ControlMessagesTuple ctrl_messages_;

    // typename rclcpp::Publisher<ControlMessageT>::SharedPtr
    // control_publisher_; typename
    // rclcpp::Subscription<StateMessageT>::SharedPtr
    // state_estimator_subscriber_;

    // In the same way that I can make a tuple of pubs and subs, I should be
    // able to make a tuple of callback types Then the downstream user will need
    // to populate the tuple with the callbacks (can be results of std::bind)
    // Then I should be able to assign the Callback to the write function by teh
    // get function on the tuple (because I know the type) If the tuple is empty
    // or the get doesn't work then it means the user did not specify the call
    // back

  private:
    template <typename MessageT> void CreatePubWithStr(const MessageT& msg) {
        // TODO: need to associate the message type with the config string
        // The config string will have a message_type field
        // All obelisk messages will have a constant string field MESSAGE_NAME
        // At runtime loop through these and use the config string that has
        // message_type = MESSAGE_NAME
        // TODO: Probably best to make a map between the message type and its
        // NAME, so the user can append more if they want
        bool message_found = false;
        for (const auto& config : config_strs_) {
            // Parse the string into a map
            const auto config_map = ParseConfigStr(config);
            // Check if the message name matches the name in a message
            if (GetMessageName(config_map) == MessageT::MESSAGE_NAME) {
                // RCLCPP_INFO_STREAM(
                // this->get_logger(),
                // "Message name:" << MessageT::MESSAGE_NAME);
                std::cout << "Message name: " << MessageT::MESSAGE_NAME
                          << std::endl;
                std::get<PublisherType<MessageT>>(control_publishers_) =
                    CreatePublisherFromConfigStr<MessageT>(config);
                message_found = true;
            }
        }

        if (!message_found) {
            throw std::runtime_error(
                "No config string contains the desired message type!");
        }
    }

    // template <typename... Args>
    // void CreatePublisher() {
    //     std::apply(CreatePubWithStr, control_publishers_);
    // }
};
} // namespace obelisk

// namespace obelisk::internal {
//     template <typename T>
//     class ObeliskController : public ObeliskNode {

//     }
// } // namespace obelisk::internal

// Chat gpt example
// For the subscriber:
// #include <iostream>
// #include <typeinfo>

// // Primary template class definition
// template<typename... Types>
// class MyClass;

// // Base case: specialization for a single type
// template<typename T>
// class MyClass<T> {
// public:
//     template<typename U>
//     void handle_type() {
//         if (typeid(T) == typeid(U)) {
//             std::cout << "Handling type: " << typeid(U).name() << std::endl;
//         } else {
//             std::cout << "Type mismatch: " << typeid(U).name() << " != " <<
//             typeid(T).name() << std::endl;
//         }
//     }
// };

// // Recursive case: handle the first type and then the rest
// template<typename T, typename... Rest>
// class MyClass<T, Rest...> : public MyClass<Rest...> {
// public:
//     using MyClass<Rest...>::handle_type; // Bring the base class function
//     into scope

//     template<typename U>
//     void handle_type() {
//         if (typeid(T) == typeid(U)) {
//             std::cout << "Handling type: " << typeid(U).name() << std::endl;
//         } else {
//             MyClass<Rest...>::template handle_type<U>();
//         }
//     }
// };

// int main() {
//     MyClass<int, double, char> myClass;

//     // Call handle_type for each type explicitly
//     myClass.handle_type<int>();    // Output: Handling type: int
//     myClass.handle_type<double>(); // Output: Handling type: double
//     myClass.handle_type<char>();   // Output: Handling type: char
//     myClass.handle_type<float>();  // Output: Type mismatch: float != char
//     (since float is not part of the types)

//     return 0;
// }

// For the publisher:
// #include <iostream>

// // Primary template class definition
// template<typename... Types>
// class MyClass;

// // Base case: specialization for a single type
// template<typename T>
// class MyClass<T> {
// public:
//     void handleType() {
//         std::cout << "Handling type: " << typeid(T).name() << std::endl;
//     }
// };

// // Recursive case: handle the first type and then the rest
// template<typename T, typename... Rest>
// class MyClass<T, Rest...> : public MyClass<Rest...> {
// public:
//     using MyClass<Rest...>::handleType; // Bring the base class function into
//     scope

//     void handleType() {
//         std::cout << "Handling type: " << typeid(T).name() << std::endl;
//         MyClass<Rest...>::handleType();
//     }
// };

// int main() {
//     MyClass<int, double, char> myClass;
//     myClass.handleType(); // Will call handleType for each type: int, double,
//     char return 0;
// }
