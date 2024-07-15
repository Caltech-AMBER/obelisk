#pragma once

#include "obelisk_viz_robot.h"

namespace obelisk::viz {
    template <typename EstimatorMessageT> class ObeliskVizRobotDefault : public ObeliskVizRobot<EstimatorMessageT> {
      public:
        explicit ObeliskVizRobotDefault(const std::string& name) : ObeliskVizRobot<EstimatorMessageT>(name) {}

      protected:
        void ParseEstimatedState(const EstimatorMessageT& msg) override {
            /*
             * Assuming the message has the following fields:
             * base_link_name: name of the base link
             * q_base: vector of (x, y, z, quat_x, quat_y, quat_z, quat_w), if length is zero then assume fixed base and
             * set default value
             * joint_names: names of all the joints
             * q_joints: vector of joint positions
             *
             * The length of "joint_names" must match the length of q_joints
             *
             * TODO: may want to consider also having a stamp field so we can get the time the information was
             * generated, not the time it was parsed for viz
             */

            // ----------- Create the transform ----------- //
            this->base_tf_.header.stamp =
                this->get_clock()->now(); // Gets the current time. Change to get the time from the message
            this->base_tf_.header.frame_id =
                "world"; // TODO: Consider making this not hardcoded. This must match the rviz fixed frame
            this->base_tf_.child_frame_id = (this->get_parameter("tf_prefix").as_string() + msg.base_link_name).c_str();

            // Check for floating vs fixed base
            if (msg.q_base.empty()) {
                this->base_tf_.transform.translation.x = 0.0;
                this->base_tf_.transform.translation.y = 0.0;
                this->base_tf_.transform.translation.z = 0.0;

                this->base_tf_.transform.rotation.x    = 0.0;
                this->base_tf_.transform.rotation.y    = 0.0;
                this->base_tf_.transform.rotation.z    = 0.0;
                this->base_tf_.transform.rotation.w    = 1.0;
            } else if (msg.q_base.size() == 7) {
                this->base_tf_.transform.translation.x = msg.q_base.at(0);
                this->base_tf_.transform.translation.y = msg.q_base.at(1);
                this->base_tf_.transform.translation.z = msg.q_base.at(2);

                tf2::Quaternion q(msg.q_base.at(3), msg.q_base.at(4), msg.q_base.at(5), msg.q_base.at(6));

                q.normalize(); // normalize the quaternion

                this->base_tf_.transform.rotation.x = q.x();
                this->base_tf_.transform.rotation.y = q.y();
                this->base_tf_.transform.rotation.z = q.z();
                this->base_tf_.transform.rotation.w = q.w();
            } else {
                throw std::runtime_error("Length of q_base in a recieved message is neither empty nor 7! Use empty for "
                                         "fixed base, and size 7 (position and quaternion) for floating base.");
            }

            // ----------- Create the JointState message ----------- //
            // Verify the message is self consistent
            if (msg.q_joints.size() != msg.joint_names.size()) {
                throw std::runtime_error("The message's q_joint and joint_names vectors have different lengths!");
            }

            // Verify the message against the URDF
            if (msg.joint_names.size() != this->model_.joints_.size()) {
                throw std::runtime_error("The message's number of joints does match the URDF!");
            }

            for (const auto& msg_joint : msg.joint_names) {
                bool valid_name = false;
                for (const auto& joint : this->model_.joints_) {
                    if (joint.first == msg_joint) {
                        valid_name = true;
                    }
                }
                if (!valid_name) {
                    RCLCPP_ERROR_STREAM(this->get_logger(),
                                        "The joint name: "
                                            << msg_joint
                                            << " found in the message does not match a joint name in the URDF!");
                    throw std::runtime_error("At least one joint name does not match!");
                }
            }

            // Construct the message
            this->joint_state_.position     = msg.q_joints;
            this->joint_state_.name         = msg.joint_names;
            this->joint_state_.header.stamp = this->base_tf_.header.stamp;
        }

      private:
    };
} // namespace obelisk::viz
