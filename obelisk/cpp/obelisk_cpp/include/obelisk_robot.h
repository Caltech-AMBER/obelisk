#pragma once

#include "obelisk_node.h"

namespace obelisk {
    /**
     * @brief Abstract Obelisk robot node.
     *
     * Obelisk robots are representations of the physical robot. They take in Obelisk control messages and can
     * optionally output Obelisk sensor messages. We expect code in this function to communicate with the low-level
     * control interface of a real system.
     *
     * This class is templated on the type of Control message it is expected to receive.
     */
    template <typename ControlMessageT> class ObeliskRobot : public ObeliskNode {
      public:
        explicit ObeliskRobot(const std::string& name, const std::string& sub_ctrl_key = "sub_ctrl")
            : ObeliskNode(name), sub_ctrl_key_(sub_ctrl_key) {

            // Register all components
            this->RegisterObkSubscription<ControlMessageT>(
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
         * @brief Activates up the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn virtual on_activate(
            const rclcpp_lifecycle::State& prev_state) {
            this->ObeliskNode::on_activate(prev_state);

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief Deactivates up the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn virtual on_deactivate(
            const rclcpp_lifecycle::State& prev_state) {
            this->ObeliskNode::on_deactivate(prev_state);

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief Cleans up the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn virtual on_cleanup(
            const rclcpp_lifecycle::State& prev_state) {
            this->ObeliskNode::on_cleanup(prev_state);

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief Shutsdown the node the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn virtual on_shutdown(
            const rclcpp_lifecycle::State& prev_state) {
            this->ObeliskNode::on_shutdown(prev_state);

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

      protected:
        /**
         * @brief Apply the control message
         *
         * Code to interface with the simulator or hardware should be implemented here.
         *
         */
        virtual void ApplyControl(const ControlMessageT& msg) = 0;

        std::string sub_ctrl_key_;

      private:
    };
} // namespace obelisk
