#include "obelisk_node.h"

namespace obelisk {

    template <typename ControlMessageT, typename EstimatorMessageT> class ObeliskController : public ObeliskNode {
      public:
        explicit ObeliskController(const std::string& name) : ObeliskNode(name) {
            // Declare all paramters
            this->declare_parameter<std::string>("timer_ctrl_setting", "");
            this->declare_parameter<std::string>("pub_ctrl_setting", "");
            this->declare_parameter<std::string>("sub_est_setting", "");
        }

        /**
         * @brief Configures all the required ROS components. Specifcially this
         * registers the control_publisher_, state_estimator_subscriber_, and the
         * control_timer_. Also makes a call to ObeliskNode on configure to parse
         * and create the callback group map.
         *
         * @param prev_state the state of the ros node.
         * @return success if everything completes.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& prev_state) {
            ObeliskNode::on_configure(prev_state);

            // Create the publishers, subscribers, and timers
            control_publisher_ =
                CreatePublisherFromConfigStr<ControlMessageT>(this->get_parameter("pub_ctrl_setting").as_string());

            state_estimator_subscriber_ = CreateSubscriptionFromConfigStr<EstimatorMessageT>(
                this->get_parameter("sub_est_setting").as_string(),
                std::bind(&ObeliskController::UpdateXHat, this, std::placeholders::_1));

            control_timer_ = CreateWallTimerFromConfigStr(this->get_parameter("timer_ctrl_setting").as_string(),
                                                          std::bind(&ObeliskController::ComputeControl, this));

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief activates the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& prev_state) {
            control_publisher_->on_activate();
            // TODO: Do anything with the timer?

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief deactivates the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& prev_state) {
            control_publisher_->on_deactivate();
            // TODO: Do anything with the timer?

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief cleans up the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State& prev_state) {
            ObeliskNode::on_cleanup(prev_state);

            // Release the shared pointers
            control_publisher_.reset();
            state_estimator_subscriber_.reset();
            control_timer_.reset();

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief shutsdown the ros node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State& prev_state) {
            ObeliskNode::on_shutdown(prev_state);

            // Release the shared pointers
            control_publisher_.reset();
            state_estimator_subscriber_.reset();
            control_timer_.reset();

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

      protected:
        /**
         * @brief Abstract method to be implemented downstream. This is
         * automatically registered as the timer callback.
         */
        virtual ControlMessageT ComputeControl() = 0;

        /**
         * @brief Abstract method to be implemented downstream. This is
         * automatically registered as the subscriber callback.
         */
        virtual void UpdateXHat(const EstimatorMessageT& msg) = 0;

        // publishes the control actions
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<ControlMessageT>> control_publisher_;

        // subscribes to the state estimate messages
        typename rclcpp::Subscription<EstimatorMessageT>::SharedPtr state_estimator_subscriber_;

        // timer to activate ComputeControl
        rclcpp::TimerBase::SharedPtr control_timer_;

      private:
    };
} // namespace obelisk
