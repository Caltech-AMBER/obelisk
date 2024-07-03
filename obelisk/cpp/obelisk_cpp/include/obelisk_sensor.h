#pragma once

#include "obelisk_node.h"

namespace obelisk {
    class ObeliskSensor : public ObeliskNode {
      public:
        explicit ObeliskSensor(const std::string& name) : ObeliskNode(name) {
            // this->declare_parameter<std::vector<std::string>>("pub_sensor_setting", {""});
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& prev_state) {
            ObeliskNode::on_configure(prev_state);

            // pub_sensor_config_strs_ = this->get_parameter("pub_sensor_setting").as_string_array();

            // // If there are no string, or just the default one, then warn the user
            // if ((!pub_sensor_config_strs_.empty() && pub_sensor_config_strs_.at(0) == "") ||
            //     pub_sensor_config_strs_.empty()) {
            //     throw std::runtime_error("No configuration strings were provided for the sensor publishers.");
            // }
            // The downstream user must create all their sensor subscribers

            // TODO (@zolkin): Will want to add automatic registration of publishers here.
            //  The key is that the publishers publish different types and thus everything is of a different type
            //  I think the way to go about this is to use tuplecat to create a tuple with all the resulting publishers.
            //  This can be done inside of a function that is called from a std::apply (the message types are already in
            //  a tuple). Then the hardpart is creating compile indicies and mapping them to the names.

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

            // Clear the config strings
            pub_sensor_config_strs_.clear();

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief shutsdown the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State& prev_state) {
            ObeliskNode::on_shutdown(prev_state);

            // Clear the config strings
            pub_sensor_config_strs_.clear();

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

      protected:
        std::vector<std::string> pub_sensor_config_strs_;

      private:
    };
} // namespace obelisk
