#pragma once

#include "obelisk_node.h"

namespace obelisk {
    template <typename ControlMessageT> class ObeliskRobot : public ObeliskNode {
      public:
        explicit ObeliskRobot(const std::string& name, const std::string& sub_ctrl_key = "sub_ctrl")
            : ObeliskNode(name), sub_ctrl_key_(sub_ctrl_key) {

            // Register all components
            this->RegisterSubscription<ControlMessageT>(
                "sub_ctrl_setting", sub_ctrl_key_, std::bind(&ObeliskRobot::ApplyControl, this, std::placeholders::_1));
        }

        /**
         * @brief Configures all the required ROS components. Specifcially this
         * registers the control_subscriver_. Also makes a call to ObeliskNode on configure
         * to parse and create the callback group map.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn virtual on_configure(
            const rclcpp_lifecycle::State& prev_state) {
            this->ObeliskNode::on_configure(prev_state);

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief activates up the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn virtual on_activate(
            const rclcpp_lifecycle::State& prev_state) {
            this->ObeliskNode::on_activate(prev_state);

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief deactivates up the node.
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
            this->ObeliskNode::on_cleanup(prev_state);

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief shutsdown the node the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn virtual on_shutdown(
            const rclcpp_lifecycle::State& prev_state) {
            this->ObeliskNode::on_shutdown(prev_state);

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

      protected:
        virtual void ApplyControl(const ControlMessageT& msg) = 0;

        std::string sub_ctrl_key_;

      private:
    };
} // namespace obelisk
