#include "rclcpp/rclcpp.hpp"

#include "obelisk_controller.h"
#include "obelisk_ros_utils.h"

namespace obelisk {
    using leap_controller_msg = obelisk_control_msgs::msg::PositionSetpoint;
    using leap_estimator_msg  = obelisk_estimator_msgs::msg::EstimatedState;

    /**
     * @brief Controller that generates a sinusoidal position setpoint for the Leap hand
     */
    class LeapPositionSetpointController : public ObeliskController<leap_controller_msg, leap_estimator_msg> {
      public:
        /**
         * @brief Construct a new Leap Position Setpoint Controller object
         *
         * @param name Name of the controller
         */
        LeapPositionSetpointController(const std::string& name)
            : ObeliskController<leap_controller_msg, leap_estimator_msg>(name) {}

      protected:
        void UpdateXHat(__attribute__((unused)) const leap_estimator_msg& msg) override {}

        leap_controller_msg ComputeControl() override {
            leap_controller_msg msg;

            msg.u_mujoco.clear();
            msg.q_des.clear();
            double time_sec = this->get_clock()->now().seconds();

            for (int i = 0; i < 16; i++) {
                msg.u_mujoco.emplace_back(amplitude_ * sin(time_sec));
                msg.q_des.emplace_back(amplitude_ * sin(time_sec));
            }

            this->GetPublisher<leap_controller_msg>(this->ctrl_key_)->publish(msg);

            return msg;
        };

        float amplitude_ = 0.3;
    };
} // namespace obelisk
