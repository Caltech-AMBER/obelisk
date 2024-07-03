#pragma once

#include "obelisk_node.h"

namespace obelisk {
    template <typename EstimatorMessageT> class ObeliskEstimator : public ObeliskNode {
      public:
        explicit ObeliskEstimator(const std::string& name) : ObeliskNode(name) {
            this->declare_parameter<std::string>("timer_est_setting", "");
            this->declare_parameter<std::string>("pub_est_setting", "");
            // this->declare_parameter<std::vector<std::string>>("sub_sensor_settings", {""});
        }

        /**
         * @brief Configures all the required ROS components. Specifcially this
         * registers the estimator_publisher_, and the estimator_timer_. Also makes a call to ObeliskNode on configure
         * to parse and create the callback group map.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& prev_state) {
            ObeliskNode::on_configure(prev_state);

            estimator_publisher_ =
                CreatePublisherFromConfigStr<EstimatorMessageT>(this->get_parameter("pub_est_setting").as_string());

            estimator_timer_ = CreateWallTimerFromConfigStr(this->get_parameter("timer_est_setting").as_string(),
                                                            std::bind(&ObeliskEstimator::ComputeStateEstimate, this));

            // The downstream user must create all their sensor subscribers using these config strings
            // sub_sensor_config_strs_ = this->get_parameter("sub_sensor_settings").as_string_array();

            // // If there are no string, or just the default one, then warn the user
            // if ((!sub_sensor_config_strs_.empty() && sub_sensor_config_strs_.at(0) == "") ||
            //     sub_sensor_config_strs_.empty()) {
            //     RCLCPP_WARN_STREAM(
            //         this->get_logger(),
            //         "No configuration strings found for the estimator subscribers (i.e. listening to sensors).");
            // }

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief activates the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& prev_state) {
            estimator_publisher_->on_activate();
            // TODO: Do anything with the timer?

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief deactivates the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& prev_state) {
            estimator_publisher_->on_deactivate();
            // TODO: Do anything with the timer?

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
            estimator_publisher_.reset();
            estimator_timer_.reset();

            // Clear the config strings
            sub_sensor_config_strs_.clear();

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief shutsdown the ros node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State& prev_state) {
            ObeliskNode::on_shutdown(prev_state);

            // Release the shared pointers
            estimator_publisher_.reset();
            estimator_timer_.reset();

            // Clear the config strings
            sub_sensor_config_strs_.clear();

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

      protected:
        /**
         * @brief Abstract method to be implemented downstream. This is
         * automatically registered as the timer callback.
         */
        virtual EstimatorMessageT ComputeStateEstimate() = 0;

        std::vector<std::string> sub_sensor_config_strs_;

        // Publishes the estimated state
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<EstimatorMessageT>> estimator_publisher_;

        // timer to activate ComputeControl
        rclcpp::TimerBase::SharedPtr estimator_timer_;

      private:
    };
} // namespace obelisk
