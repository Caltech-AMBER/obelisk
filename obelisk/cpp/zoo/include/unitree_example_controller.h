#include "rclcpp/rclcpp.hpp"

#include "obelisk_controller.h"
#include "obelisk_ros_utils.h"

namespace obelisk {
    using unitree_controller_msg = obelisk_control_msgs::msg::PDFeedForward;
    using unitree_estimator_msg  = obelisk_estimator_msgs::msg::EstimatedState;

    /**
     * @brief Controller that generates a sinusoidal position setpoint for the Leap hand
     */
    class UnitreeExampleController : public ObeliskController<unitree_controller_msg, unitree_estimator_msg> {
      public:
        /**
         * @brief Construct a new Leap Position Setpoint Controller object
         *
         * @param name Name of the controller
         */
        UnitreeExampleController(const std::string& name)
            : ObeliskController<unitree_controller_msg, unitree_estimator_msg>(name) {}

      protected:
        void UpdateXHat(__attribute__((unused)) const unitree_estimator_msg& msg) override {}

        unitree_controller_msg ComputeControl() override {
            unitree_controller_msg msg;

            msg.u_mujoco.clear();
            msg.pos_target.clear();
            double time_sec = this->get_clock()->now().seconds();

            for (int i = 0; i < 43; i++) {
                msg.u_mujoco.emplace_back(amplitude_ * sin(time_sec));
                msg.pos_target.emplace_back(amplitude_ * sin(time_sec));
            }

            for (int i = 0; i < 2*43; i++) {
                msg.u_mujoco.emplace_back(0);
            }

            this->GetPublisher<unitree_controller_msg>(this->ctrl_key_)->publish(msg);

            return msg;
        };

        float amplitude_ = 0.3;
    };
} // namespace obelisk
