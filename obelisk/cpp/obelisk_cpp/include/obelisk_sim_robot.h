#pragma once

#include "obelisk_robot.h"
#include "obelisk_sensor_msgs/msg/true_sim_state.hpp"

namespace obelisk {
    template <typename ControlMessageT> class ObeliskSimRobot : public ObeliskRobot<ControlMessageT> {
        using TrueSimState = obelisk_sensor_msgs::msg::TrueSimState;

      public:
        explicit ObeliskSimRobot(const std::string& name) : ObeliskRobot<ControlMessageT>(name) {
            this->template declare_parameter<std::string>("timer_true_sim_state_setting", "");
            this->template declare_parameter<std::string>("pub_true_sim_state_setting", "");
            stop_thread_ = false;
        }

        /**
         * @brief Configures all the required ROS components. Specifcially this
         * registers the true_sim_state_publisher_ if a configuration is passed. Also makes a call to ObeliskNode on
         * configure to parse and create the callback group map.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn virtual on_configure(
            const rclcpp_lifecycle::State& prev_state) {
            this->ObeliskRobot<ControlMessageT>::on_configure(prev_state);

            const std::string timer_settings = this->get_parameter("timer_true_sim_state_setting").as_string();
            const std::string pub_settings   = this->get_parameter("pub_true_sim_state_setting").as_string();

            if (timer_settings != "" && pub_settings != "") {
                true_sim_state_publisher_ = this->template CreatePublisherFromConfigStr<TrueSimState>(pub_settings);
                true_sim_state_timer_ =
                    this->CreateWallTimerFromConfigStr(this->get_parameter("timer_true_sim_state_setting").as_string(),
                                                       std::bind(&ObeliskSimRobot::PublishTrueSimState, this));
                true_sim_state_timer_->cancel(); // Prevent the timer from starting instantly
            } else {
                RCLCPP_WARN_STREAM(this->get_logger(), "No true simulation state timer/publisher configured.");
                true_sim_state_timer_     = nullptr;
                true_sim_state_publisher_ = nullptr;
            }

            stop_thread_ = false;

            // Start the simulator
            sim_thread_ = std::thread(&ObeliskSimRobot::StartSim, this);
            RCLCPP_INFO_STREAM(this->get_logger(), "Simulation thread started.");

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief activates up the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn virtual on_activate(
            const rclcpp_lifecycle::State& prev_state) {
            this->ObeliskRobot<ControlMessageT>::on_activate(prev_state);

            if (true_sim_state_publisher_) {
                true_sim_state_publisher_->on_activate();
            }

            if (true_sim_state_timer_) {
                true_sim_state_timer_->reset(); // Start the timer
            }

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief deactivates up the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn virtual on_deactivate(
            const rclcpp_lifecycle::State& prev_state) {
            this->ObeliskRobot<ControlMessageT>::on_deactivate(prev_state);
            if (true_sim_state_publisher_) {
                true_sim_state_publisher_->on_deactivate();
            }

            if (true_sim_state_timer_) {
                true_sim_state_timer_->cancel(); // Stop the timer
            }

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief cleans up the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn virtual on_cleanup(
            const rclcpp_lifecycle::State& prev_state) {
            this->ObeliskRobot<ControlMessageT>::on_cleanup(prev_state);

            // Release the shared pointers
            if (true_sim_state_timer_) {
                true_sim_state_timer_->cancel();
                true_sim_state_timer_.reset(); // release the timer
            }

            if (true_sim_state_publisher_) {
                true_sim_state_publisher_.reset();
            }

            // Cleanup the sim thread
            bool current_thread_status = stop_thread_;
            stop_thread_               = true;
            if (sim_thread_.joinable()) {
                sim_thread_.join();
            }

            if (!current_thread_status) {
                RCLCPP_INFO_STREAM(this->get_logger(), "Simulation thread stopped.");
            }

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief shutsdown the node the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn virtual on_shutdown(
            const rclcpp_lifecycle::State& prev_state) {
            this->ObeliskRobot<ControlMessageT>::on_shutdown(prev_state);

            return on_cleanup(prev_state);
        }

      protected:
        /**
         * @brief Publish the TrueSimState of the simulator.
         *
         * This is the timer callback that publishes the TrueSimState and is expected to call
         * self.publisher_true_sim_state internally. Note that the true sim state message is still returned afterwards,
         * mostly for logging/debugging purposes. The publish call is the important part, NOT the returned value, since
         * the topic is what the ObeliskEstimator subscribes to.
         *
         * @return obelisk_true_sim_state_msg: An Obelisk message type containing the true sim state.
         */
        virtual TrueSimState PublishTrueSimState() = 0;

        /**
         * @brief Run the simulator
         *
         * The control input into the simulator is accessed through the shared control array. If any simulator
         * configuration needs to occur, then the on_configure function should be overridden. This function should run
         * the simulator loop and update the state of the simulator as long as the node is active.
         *
         * This is where the simulation loop is run. The simulation loop should always be check the stop_thread_ flag to
         * determine when to stop.
         */
        virtual void RunSimulator() = 0;

        void StartSim() { this->RunSimulator(); }

        // ---------- Member Variables --------- //
        // Number of control inputs
        // int n_u_;

        // Publisher for the true sim state
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<TrueSimState>> true_sim_state_publisher_;

        // Timer to publish the true sim state
        rclcpp::TimerBase::SharedPtr true_sim_state_timer_;

        // Hold the thread where the simulation takes place
        std::thread sim_thread_;

        // Flag to signal stopping the thread
        std::atomic<bool> stop_thread_;

      private:
    };
} // namespace obelisk