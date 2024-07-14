#pragma once

#include <fstream>
#include <iostream>
#include <sstream>
#include <urdf/model.h>

#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "obelisk_node.h"

//

namespace obelisk::viz {
    template <typename EstimatorMessageT> class ObeliskVizRobot : public ObeliskNode {
      public:
        explicit ObeliskVizRobot(const std::string& name, const std::string& est_key = "sub_est",
                                 const std::string& timer_key = "time_joints")
            : ObeliskNode(name), est_key_(est_key), timer_key_(timer_key) {
            urdf_path_param_ = name + "urdf_path_param";
            this->declare_parameter(urdf_path_param_, "");

            pub_settings_ = name + "pub_viz_joint_settings";
            this->declare_parameter<std::string>(pub_settings_, "");

            timer_settings_ = name + "timer_viz_joint_settings";
            this->RegisterObkTimer(timer_settings_, timer_key_, std::bind(&ObeliskVizRobot::PublishJointState, this));

            sub_settings_ = name + "sub_viz_est_settings";
            this->RegisterObkSubscription<EstimatorMessageT>(
                sub_settings_, est_key_,
                std::bind(&ObeliskVizRobot::ParseEstimatedStateSafe, this, std::placeholders::_1));

            // Create the tf broadcaster
            base_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }

        /**
         * @brief Configures the node.
         *
         * @param prev_state the state of the ros node.
         * @return success if everything completes.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& prev_state) override {
            ObeliskNode::on_configure(prev_state);

            std::string urdf_file = this->get_parameter(urdf_path_param_).as_string();
            std::ifstream file(urdf_file);
            if (!file) {
                throw std::runtime_error("Could not open URDF!");
            }

            std::stringstream buffer;
            buffer << file.rdbuf();
            std::string urdf_str = buffer.str();

            urdf::Model model;
            if (!model.initString(urdf_str)) {
                throw std::runtime_error("Could not parse URDF!");
            }

            RCLCPP_INFO_STREAM(this->get_logger(), "Successfully parsed URDF file.");
            RCLCPP_INFO_STREAM(this->get_logger(), "Robot name: " << model.getName());

            // Create the subscriber has a valid message type
            // TODO (@zolkin): find a better way to do this
            bool has_estimator_sub = false;
            using internal::estimator_message_names;
            const std::string* name_ptr = std::find(estimator_message_names.begin(), estimator_message_names.end(),
                                                    this->registered_subscriptions_[est_key_].msg_type);
            if (name_ptr != estimator_message_names.end()) {
                has_estimator_sub = true;
            }

            if (!has_estimator_sub) {
                throw std::runtime_error("Subscription message must be of type ObeliskEstimatorMsg!");
            }

            // Use the original lifecycle create function to avoid the warning, as we know we need non-obelisk here.
            // The user can only configure the topic name and history depth.
            // *** Note: The launch file must re-map the corresponding `robot_state_publisher`
            //     topic to be this topic, otherwise the topics won't match. ***
            auto config_map = ParseConfigStr(this->get_parameter(pub_settings_).as_string());
            js_publisher_   = rclcpp_lifecycle::LifecycleNode::create_publisher<sensor_msgs::msg::JointState>(
                GetTopic(config_map), GetHistoryDepth(config_map));

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief Activates the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& prev_state) override {
            this->ObeliskNode::on_activate(prev_state);
            if (js_publisher_) {
                js_publisher_->on_activate();
            }

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief Deactivates the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& prev_state) override {
            this->ObeliskNode::on_deactivate(prev_state);
            if (js_publisher_) {
                js_publisher_->on_deactivate();
            }

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief Cleans up the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State& prev_state) override {
            ObeliskNode::on_cleanup(prev_state);
            if (js_publisher_) {
                js_publisher_.reset();
            }

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief Shuts down the ros node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State& prev_state) override {
            ObeliskNode::on_shutdown(prev_state);
            if (js_publisher_) {
                js_publisher_.reset();
            }

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

      protected:
        void ParseEstimatedStateSafe(const EstimatorMessageT& msg) {
            // Start a lock guard here to ensure safety
            const std::lock_guard<std::mutex> lock(state_mut_);

            this->ParseEstimatedState(msg);
        }

        /**
         * @brief Must parse the estimated state into a JointState message and a TransformedStamped
         * joint_state_ and base_tf_ must be populated by this function.
         */
        virtual void ParseEstimatedState(const EstimatorMessageT& msg) = 0;

        void PublishJointState() {
            // Secture the mutex
            const std::lock_guard<std::mutex> lock(state_mut_);

            // Publish
            js_publisher_->publish(joint_state_);
            base_broadcaster_->sendTransform(base_tf_);
        }

        // Publisher
        rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr js_publisher_;

        // Need a mutex for the common joint state message
        std::mutex state_mut_;

        // Hold a joint state message
        sensor_msgs::msg::JointState joint_state_;
        geometry_msgs::msg::TransformStamped base_tf_;

        // TF broadcaster to broadcase the location of the base frame in the world
        std::unique_ptr<tf2_ros::TransformBroadcaster> base_broadcaster_;

        std::string timer_key_;
        std::string est_key_;
        std::string urdf_path_param_;

        std::string pub_settings_;
        std::string sub_settings_;
        std::string timer_settings_;

      private:
    };
} // namespace obelisk::viz
