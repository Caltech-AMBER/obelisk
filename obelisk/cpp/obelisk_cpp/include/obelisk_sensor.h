#pragma once

#include "obelisk_node.h"

namespace obelisk {
    class ObeliskSensor : public ObeliskNode {
      public:
        explicit ObeliskSensor(const std::string& name) : ObeliskNode(name) { has_sensor_pub_ = false; }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn virtual on_configure(
            const rclcpp_lifecycle::State& prev_state) {
            ObeliskNode::on_configure(prev_state);

            // TODO (@zolkin): find a better way to do this
            using internal::sensor_message_names;
            for (const auto [key, reg_pub] : registered_publishers_) {
                const std::string* name_ptr =
                    std::find(sensor_message_names.begin(), sensor_message_names.end(), reg_pub.msg_type);
                if (name_ptr != sensor_message_names.end()) {
                    has_sensor_pub_ = true;
                }
            }

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
