#include "obelisk_node.h"

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
template <typename ControlMessagesT, typename EstimatorMessagesT>
class ObeliskController : public ObeliskNode {
  public:
    explicit ObeliskController(const std::string& name) : ObeliskNode(name) {
        // Declare all paramters
        this->declare_parameter("callback_group_config_strs", "topic:topc1");
        this->declare_parameter("timer_ctrl_config_str", "topic:topc1");
        this->declare_parameter("pub_ctrl_config_str", "topic:topic1");
        this->declare_parameter("sub_est_config_str", "topic:topic1");
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
        std::apply(CreatePubWithStr, control_publishers_);
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
    template <typename MessageT>
    void CreatePubWithStr(PublisherType<MessageT>& publisher) {
        publisher = CreatePublisherFromConfigStr<MessageT>(
            this->get_parameter("pub_ctrl_config_str"));
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
