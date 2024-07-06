#pragma once

#include "obelisk_node.h"

namespace obelisk {

    template <typename ControlMessageT, typename EstimatorMessageT> class ObeliskController : public ObeliskNode {
      public:
        explicit ObeliskController(const std::string& name, const std::string& ctrl_key = "pub_ctrl",
                                   const std::string& est_key = "sub_est", const std::string& timer_key = "timer_ctrl")
            : ObeliskNode(name), ctrl_key_(ctrl_key), est_key_(est_key), timer_key_(timer_key) {

            // Register all components
            this->RegisterTimer("timer_ctrl_setting", timer_key_, std::bind(&ObeliskController::ComputeControl, this));
            this->RegisterPublisher<ControlMessageT>("pub_ctrl_setting", ctrl_key_);
            this->RegisterSubscription<EstimatorMessageT>(
                "sub_est_setting", est_key_, std::bind(&ObeliskController::UpdateXHat, this, std::placeholders::_1));
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
        virtual ControlMessageT ComputeControl() = 0;

        /**
         * @brief Abstract method to be implemented downstream. This is
         * automatically registered as the subscriber callback.
         */
        virtual void UpdateXHat(const EstimatorMessageT& msg) = 0;

        const std::string ctrl_key_;
        const std::string est_key_;
        const std::string timer_key_;

      private:
    };
} // namespace obelisk
