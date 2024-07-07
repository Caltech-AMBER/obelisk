#pragma once

#include "obelisk_node.h"

namespace obelisk {
    template <typename EstimatorMessageT> class ObeliskEstimator : public ObeliskNode {
      public:
        explicit ObeliskEstimator(const std::string& name, const std::string& est_pub_key = "publisher_est",
                                  const std::string& est_timer_key = "timer_est_setting")
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
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn virtual on_configure(
            const rclcpp_lifecycle::State& prev_state) {
            ObeliskNode::on_configure(prev_state);

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief activates the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn virtual on_activate(
            const rclcpp_lifecycle::State& prev_state) {
            this->ObeliskNode::on_activate(prev_state);

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief deactivates the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn virtual on_deactivate(
            const rclcpp_lifecycle::State& prev_state) {
            this->ObeliskNode::on_deactivate(prev_state);

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief cleans up the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn virtual on_cleanup(
            const rclcpp_lifecycle::State& prev_state) {
            ObeliskNode::on_cleanup(prev_state);

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief shutsdown the ros node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn virtual on_shutdown(
            const rclcpp_lifecycle::State& prev_state) {
            ObeliskNode::on_shutdown(prev_state);

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

      protected:
        /**
         * @brief Abstract method to be implemented downstream. This is
         * automatically registered as the timer callback.
         */
        virtual EstimatorMessageT ComputeStateEstimate() = 0;

        std::string est_pub_key_;
        std::string est_timer_key_;

      private:
    };
} // namespace obelisk
