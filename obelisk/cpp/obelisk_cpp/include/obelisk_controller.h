#include "obelisk_node.h"
// #include "rclcpp_lifecycle/lifecycle_node_interface.hpp"

namespace obelisk {
// template <typename... MessagesT>
// TODO: Can make two of these with different names, but do pretty much the same
// thing (one for pub, one for sub)
// struct MessagePack {
//     using MessagePackType = std::tuple<MessagesT...>;
//     // using PublisherPack = std::tuple<>
//     using PublisherPackType =
//         std::tuple<typename rclcpp::Publisher<MessagesT>::SharedPtr...>;
//     using SubscriberPackType =
//         std::tuple<typename rclcpp::Subscription<MessagesT>::SharedPtr...>;
//     using SubscriberCallbackPackType =
//         std::tuple<std::function<void(MessagesT)>...>;
//     using TimerCallbackPackType =
//     std::tuple<std::function<MessagesT(void)>...>;
// };

// TODO: Consider multiple types here (variadic)
template <typename ControlMessagesT, typename EstimatorMessagesT>
class ObeliskController : public ObeliskNode {
  public:
    explicit ObeliskController(const std::string& name) : ObeliskNode(name) {
        // Declare all paramters
        this->declare_parameter<std::string>("timer_ctrl_config_str");
        this->declare_parameter<std::string>("pub_ctrl_config_str");
        this->declare_parameter<std::string>("sub_est_config_str");
        // "topic:topic1,message_type:PositionSetpoint"
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State& prev_state) {
        // Create the publishers, subscribers, and timers
        control_publisher_ = CreatePublisherFromConfigStr<ControlMessagesT>(
            this->get_parameter("pub_ctrl_config_str").as_string());

        state_estimator_subscriber_ =
            CreateSubscriptionFromConfigStr<EstimatorMessagesT>(
                this->get_parameter("sub_est_config_str").as_string(),
                std::bind(&ObeliskController::SimpleSubCB<EstimatorMessagesT>,
                          this, std::placeholders::_1));

        control_timer_ = CreateWallTimerFromConfigStr(
            this->get_parameter("timer_ctrl_config_str").as_string(),
            std::bind(&ObeliskController::SimpleTimerCB, this));

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

  protected:
    void SimpleTimerCB() {}
    template <typename MessageT> void SimpleSubCB(const MessageT& msg) {}

    typename rclcpp::Publisher<ControlMessagesT>::SharedPtr control_publisher_;
    typename rclcpp::Subscription<EstimatorMessagesT>::SharedPtr
        state_estimator_subscriber_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // In the same way that I can make a tuple of pubs and subs, I should be
    // able to make a tuple of callback types Then the downstream user will need
    // to populate the tuple with the callbacks (can be results of std::bind)
    // Then I should be able to assign the Callback to the write function by teh
    // get function on the tuple (because I know the type) If the tuple is empty
    // or the get doesn't work then it means the user did not specify the call
    // back

  private:
    // template <typename MessageT> void CreatePubWithStr(const MessageT& msg) {
    //     // TODO: need to associate the message type with the config string
    //     // The config string will have a message_type field
    //     // All obelisk messages will have a constant string field
    //     MESSAGE_NAME
    //     // At runtime loop through these and use the config string that has
    //     // message_type = MESSAGE_NAME
    //     // TODO: Probably best to make a map between the message type and its
    //     // NAME, so the user can append more if they want
    //     bool message_found = false;
    //     for (const auto& config : config_strs_) {
    //         // Parse the string into a map
    //         const auto config_map = ParseConfigStr(config);
    //         // Check if the message name matches the name in a message
    //         if (GetMessageName(config_map) == MessageT::MESSAGE_NAME) {
    //             // RCLCPP_INFO_STREAM(
    //             // this->get_logger(),
    //             // "Message name:" << MessageT::MESSAGE_NAME);
    //             std::cout << "Message name: " << MessageT::MESSAGE_NAME
    //                       << std::endl;
    //             std::get<PublisherType<MessageT>>(control_publishers_) =
    //                 CreatePublisherFromConfigStr<MessageT>(config);
    //             message_found = true;
    //         }
    //     }

    //     if (!message_found) {
    //         throw std::runtime_error(
    //             "No config string contains the desired message type!");
    //     }
    // }

    // template <typename... Args>
    // void CreatePublisher() {
    //     std::apply(CreatePubWithStr, control_publishers_);
    // }
};
} // namespace obelisk
