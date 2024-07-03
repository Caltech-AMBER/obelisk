#pragma once

#include "obelisk_node.h"

namespace obelisk {
    template <typename ControlMessageT> class ObeliskRobot : public ObeliskNode {
      public:
        explicit ObeliskRobot(const std::string& name) : ObeliskNode(name) {
            this->declare_parameter<std::string>("sub_ctrl_setting", "");
            // this->declare_parameter<std::vector<std::string>>("pub_sensor_settings", {""});
        }

        /**
         * @brief Configures all the required ROS components. Specifcially this
         * registers the control_subscriver_. Also makes a call to ObeliskNode on configure
         * to parse and create the callback group map.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& prev_state) {
            ObeliskNode::on_configure(prev_state);

            // Create the subscriber to the control input
            control_subscriber_ = CreateSubscriptionFromConfigStr<ControlMessageT>(
                this->get_parameter("sub_ctrl_setting").as_string(),
                std::bind(&ObeliskRobot::ApplyControl, this, std::placeholders::_1));

            // The downstream user must create all their sensor subscribers using these config strings
            // pub_sensor_config_strs_ = this->get_parameter("pub_sensor_setting").as_string_array();

            // If there are no string, or just the default one, then warn the user
            // if ((!pub_sensor_config_strs_.empty() && pub_sensor_config_strs_.at(0) == "") ||
            //     pub_sensor_config_strs_.empty()) {
            //     RCLCPP_WARN_STREAM(this->get_logger(),
            //                        "No configuration strings found for the robot sensor publishers.");
            // }

            // For now the downstream user needs to register the sensor publishers
            // TODO (@zolkin): Implement registration of all these publishers

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

            // Release the shared pointers
            control_subscriber_.reset();

            // Clear the config strings
            pub_sensor_config_strs_.clear();

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief shutsdown the node the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State& prev_state) {
            ObeliskNode::on_shutdown(prev_state);

            // Release the shared pointers
            control_subscriber_.reset();

            // Clear the config strings
            pub_sensor_config_strs_.clear();

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

      protected:
        virtual void ApplyControl(const ControlMessageT& msg) = 0;

        // Publishes the estimated state
        typename rclcpp::Subscription<ControlMessageT>::SharedPtr control_subscriber_;

        std::vector<std::string> pub_sensor_config_strs_;

      private:
    };
} // namespace obelisk
