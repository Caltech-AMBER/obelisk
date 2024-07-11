#pragma once

#include "obelisk_node.h"

namespace obelisk {

    /**
     *  @brief Abstract Obelisk controller node.
     *
     * Obelisk controllers are stateful. That is, all the quantities required to compute the control signal are stored
     * in the controller object itself. This is done because the processes generating each of these quantities may act
     * asynchronously. Similarly, the control action may be queried asynchronously.
     *
     * When implementing a new ObeliskController, the user should declare all quantities required to compute the control
     * input in on_configure. These quantities should be updated by various UpdateX methods. Finally, the
     * ComputeControl method should be implemented to compute the control signal using the updated quantities. Note that
     * the control message should be of type ObeliskControlMsg to be compatible with the Obelisk ecosystem.
     *
     * The class has two template parameters: one for the control message type that the node publishes
     * and one for the estimated message type that the node recieves.
     */
    template <typename ControlMessageT, typename EstimatorMessageT> class ObeliskController : public ObeliskNode {
      public:
        explicit ObeliskController(const std::string& name, const std::string& ctrl_key = "pub_ctrl",
                                   const std::string& est_key = "sub_est", const std::string& timer_key = "timer_ctrl")
            : ObeliskNode(name), ctrl_key_(ctrl_key), est_key_(est_key), timer_key_(timer_key) {

            // Register all components
            this->RegisterObkTimer("timer_ctrl_setting", timer_key_,
                                   std::bind(&ObeliskController::ComputeControl, this));
            this->RegisterObkPublisher<ControlMessageT>("pub_ctrl_setting", ctrl_key_);
            this->RegisterObkSubscription<EstimatorMessageT>(
                "sub_est_setting", est_key_, std::bind(&ObeliskController::UpdateXHat, this, std::placeholders::_1));
        }

        /**
         * @brief Configures the node.
         *
         * @param prev_state the state of the ros node.
         * @return success if everything completes.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& prev_state) final {
            ObeliskNode::on_configure(prev_state);
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief Activates the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& prev_state) final {
            this->ObeliskNode::on_activate(prev_state);
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief Deactivates the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& prev_state) final {
            this->ObeliskNode::on_deactivate(prev_state);
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief Cleans up the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State& prev_state) final {
            ObeliskNode::on_cleanup(prev_state);
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief Shuts down the ros node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State& prev_state) final {
            ObeliskNode::on_shutdown(prev_state);
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

      protected:
        /**
         * @brief Compute the control signal.
         *
         * Abstract method to be implemented downstream. This is
         * automatically registered as the timer callback. The expection is that this function is where the publisher is
         * called.
         *
         * The publish call is the important part, NOT the returned value, since the topic is what the ObeliskRobot
         * subscribes to.
         *
         * @return An Obelisk message type containing the control signal and relevant metadata.
         *
         */
        virtual ControlMessageT ComputeControl() = 0;

        /**
         * @brief Update the state estimate
         *
         * Abstract method to be implemented downstream. This is
         * automatically registered as the subscriber callback.
         *
         * @param msg: the Obelisk message containing the state estimate.
         */
        virtual void UpdateXHat(const EstimatorMessageT& msg) = 0;

        const std::string ctrl_key_;
        const std::string est_key_;
        const std::string timer_key_;

      private:
    };
} // namespace obelisk
