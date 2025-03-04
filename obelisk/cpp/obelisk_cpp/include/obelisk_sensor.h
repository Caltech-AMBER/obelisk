#pragma once

#include "obelisk_node.h"

namespace obelisk {
    /**
     * @brief Obelisk sensor node
     *
     * Obelisk sensors interface directly with sensing hardware. This could mean that this node runs from the robot,
     * runs from some offboard computer which connects to the sensors, or anything else. ObeliskSensors don't nominally
     * need to subscribe to any topics. They simply expect to publish some number of sensor messages.
     *
     */
    class ObeliskSensor : public ObeliskNode {
      public:
        explicit ObeliskSensor(const std::string& name) : ObeliskNode(name) { has_sensor_pub_ = false; }

        /**
         * @brief Configures the node.
         * Verifies that at least one registered publisher is publishing an Obelisk sensor message.
         *
         * @param prev_state the previous state of the system
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& prev_state) {
            ObeliskNode::on_configure(prev_state);

            has_sensor_pub_ = false;

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief Activates the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& prev_state) {
            this->ObeliskNode::on_activate(prev_state);
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief Deactivates the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& prev_state) {
            this->ObeliskNode::on_deactivate(prev_state);
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief Cleans up the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State& prev_state) {
            this->ObeliskNode::on_cleanup(prev_state);

            has_sensor_pub_ = false;

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief Shutsdown the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State& prev_state) {
            this->ObeliskNode::on_shutdown(prev_state);

            has_sensor_pub_ = false;

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

      protected:
        bool has_sensor_pub_;

      private:
    };
} // namespace obelisk
