#pragma once

#include "obelisk_node.h"

namespace obelisk {
    class ObeliskSensor : public ObeliskNode {
      public:
        explicit ObeliskSensor(const std::string& name) : ObeliskNode(name) { has_sensor_pub_ = false; }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn virtual on_configure(
            const rclcpp_lifecycle::State& prev_state) {
            ObeliskNode::on_configure(prev_state);

            // TODO (@zolkin): Use the MESSAGE_NAME field (stored in info.msg_type in registered_publishers_) to
            // determine if the publisher is publishing a sensor for now, just going to check if its empty
            has_sensor_pub_ = !registered_publishers_.empty();

            if (!has_sensor_pub_) {
                throw std::runtime_error("Need a sensor publisher in an Obelisk Sensor Node!");
            }

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        // TODO: Remove when we remove all need for the super call (see issue #35).
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
         * @brief shutsdown the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn virtual on_shutdown(
            const rclcpp_lifecycle::State& prev_state) {
            ObeliskNode::on_shutdown(prev_state);

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

      protected:
        bool has_sensor_pub_;

      private:
    };
} // namespace obelisk
