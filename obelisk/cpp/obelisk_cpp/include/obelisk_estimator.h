#pragma once

#include "obelisk_node.h"

namespace obelisk {
    /**
     * @brief Abstract Obelisk estimator node
     *
     * Obelisk estimators are stateful. That is, all the quantities required to compute the estimate are stored in the
     * estimator object itself. This is done because the processes generating each of these quantities may act
     * asynchronously. Similarly, the state estimate may be queried asynchronously.
     *
     * When implementing a new ObeliskEstimator, the user should declare all quantities required to compute the state
     * estimate in on_configure. These quantities should be updated by various updateX methods. Finally, the
     * compute_estimate method should be implemented to compute the state estimate using the updated quantities. Note
     * that the estimate message should be of type ObeliskEstimatorMsg to be compatible with the Obelisk ecosystem.
     *
     * This class is templated on the Estimated message type that is published.
     */
    template <typename EstimatorMessageT> class ObeliskEstimator : public ObeliskNode {
      public:
        explicit ObeliskEstimator(const std::string& name, const std::string& est_pub_key = "pub_est",
                                  const std::string& est_timer_key = "timer_est")
            : ObeliskNode(name), est_pub_key_(est_pub_key), est_timer_key_(est_timer_key) {

            // Register all components
            this->RegisterPublisher<EstimatorMessageT>("pub_est_setting", est_pub_key_);
            this->RegisterTimer("timer_est_setting", est_timer_key_,
                                std::bind(&ObeliskEstimator::ComputeStateEstimate, this));
        }

        /**
         * @brief Configures all the required ROS components. Specifcially this
         * registers the estimator_publisher_, and the estimator_timer_. Also makes a call to ObeliskNode on configure
         * to parse and create the callback group map.
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
         * @brief Shutsdown the ros node.
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
         * @brief Compute the state estimate
         *
         * Abstract method to be implemented downstream. This is
         * automatically registered as the timer callback. The expection is that this function is where the publisher is
         * called.
         *
         * The publish call is the important part, NOT the returned value, since the topic is what the ObeliskRobot
         * subscribes to.
         *
         * @return the state estimate message
         */
        virtual EstimatorMessageT ComputeStateEstimate() = 0;

        std::string est_pub_key_;
        std::string est_timer_key_;

      private:
    };
} // namespace obelisk
