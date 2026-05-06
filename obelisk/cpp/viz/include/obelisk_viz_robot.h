#pragma once

#include <fstream>
#include <iostream>
#include <sstream>
#include <urdf/model.h>

#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "obelisk_node.h"

namespace obelisk::viz {
    /**
     * @brief Abstract class to help display a robot model in RViz
     *
     * This class listens to Obelisk estimator messages and converts them to messages needed by RViz and a
     * robot_state_publisher to display a robot in the visualizer.
     *
     * Ultimately this class must publish the joint states and transform from the fixed frame to a base link.
     *
     * This class is templated on the estimator message that it is listening for.
     */
    template <typename EstimatorMessageT> class ObeliskVizRobot : public ObeliskNode {
      public:
        ObeliskVizRobot(const std::string& name, const std::string& est_key = "sub_est",
                        const std::string& timer_key = "time_joints",
                        const std::string& pub_viz_joint_key = "pub_viz_joint")
            : ObeliskNode(name), est_key_(est_key), timer_key_(timer_key),
              pub_viz_joint_key_(pub_viz_joint_key), first_message_received_(false) {
            this->declare_parameter("urdf_path_param", "");
            this->declare_parameter("tf_prefix", "");

            this->RegisterObkTimer(timer_key_, std::bind(&ObeliskVizRobot::PublishJointState, this));
            this->RegisterObkSubscription<EstimatorMessageT>(
                est_key_, std::bind(&ObeliskVizRobot::ParseEstimatedStateSafe, this, std::placeholders::_1));
            this->template RegisterObkPublisher<sensor_msgs::msg::JointState>(pub_viz_joint_key_);

            // Create the tf broadcaster
            base_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }

        /**
         * @brief Configures the node — load URDF and let ObeliskNode wire up pubs/subs/timers.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& prev_state) override {
            ObeliskNode::on_configure(prev_state);

            std::string urdf_file = this->get_parameter("urdf_path_param").as_string();
            std::ifstream file(urdf_file);
            if (!file) {
                throw std::runtime_error("Could not open URDF!");
            }

            std::stringstream buffer;
            buffer << file.rdbuf();
            std::string urdf_str = buffer.str();

            if (!model_.initString(urdf_str)) {
                throw std::runtime_error("Could not parse URDF!");
            }

            RCLCPP_INFO_STREAM(this->get_logger(), "Successfully parsed URDF file.");
            RCLCPP_INFO_STREAM(this->get_logger(), "Robot name: " << model_.getName());

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

      protected:
        void ParseEstimatedStateSafe(const EstimatorMessageT& msg) {
            // Start a lock guard here to ensure safety
            const std::lock_guard<std::mutex> lock(state_mut_);

            this->ParseEstimatedState(msg);

            first_message_received_ = true;
            RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "First state estimate message received.");
        }

        /**
         * @brief Abstract method to parse the estimated state into a JointState message and a TransformedStamped.
         * joint_state_ and base_tf_ must be populated by this function.
         *
         * @param msg the estimator message
         */
        virtual void ParseEstimatedState(const EstimatorMessageT& msg) = 0;

        /**
         * @brief Publishes the joint state and base transform.
         * Only publishes afer the first estimated state message has been received.
         */
        void PublishJointState() {
            // Secure the mutex
            const std::lock_guard<std::mutex> lock(state_mut_);

            // Publish only once we have received a message
            if (first_message_received_) {
                this->template GetPublisher<sensor_msgs::msg::JointState>(pub_viz_joint_key_)->publish(joint_state_);
                base_broadcaster_->sendTransform(base_tf_);
            }
        }

        // Need a mutex for the common joint state message
        std::mutex state_mut_;

        // This is written to in ParseEstimatedState and published in PublishJointState
        sensor_msgs::msg::JointState joint_state_;

        // This is written to in ParseEstimatedState and published in PublishJointState
        geometry_msgs::msg::TransformStamped base_tf_;

        // TF broadcaster to broadcase the location of the base frame in the world
        std::unique_ptr<tf2_ros::TransformBroadcaster> base_broadcaster_;

        std::string est_key_;
        std::string timer_key_;
        std::string pub_viz_joint_key_;

        bool first_message_received_;

        urdf::Model model_;

      private:
    };
} // namespace obelisk::viz
