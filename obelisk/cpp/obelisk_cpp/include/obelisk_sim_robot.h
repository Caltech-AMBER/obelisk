#pragma once

#include "obelisk_robot.h"
#include "obelisk_sensor_msgs/msg/true_sim_state.hpp"

namespace obelisk {
    template <typename ControlMessageT> class ObeliskSimRobot : public ObeliskRobot<ControlMessageT> {
        using TrueSimState = obelisk_sensor_msgs::msg::TrueSimState;

      public:
        explicit ObeliskSimRobot(const std::string& name, const std::string& state_timer_key = "timer_true_sim_state",
                                 const std::string& state_pub_key = "publisher_true_sim_state")
            : ObeliskRobot<ControlMessageT>(name), state_timer_key_(state_timer_key), state_pub_key_(state_pub_key) {

            // Register Components
            this->RegisterTimer("timer_true_sim_state_setting", state_timer_key_,
                                std::bind(&ObeliskSimRobot::PublishTrueSimState, this));
            this->template RegisterPublisher<TrueSimState>("pub_true_sim_state_setting", state_pub_key_);

            stop_thread_ = false;
        }

        /**
         * @brief Configures all the required ROS components. specifically this
         * registers the true_sim_state_publisher_ if a configuration is passed. Also makes a call to ObeliskNode on
         * configure to parse and create the callback group map.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn virtual on_configure(
            const rclcpp_lifecycle::State& prev_state) {
            this->ObeliskRobot<ControlMessageT>::on_configure(prev_state);

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

            EndSimThread();

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

            EndSimThread();

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

      protected:
        void EndSimThread() {
            // Cleanup the sim thread
            bool current_thread_status = stop_thread_;
            stop_thread_               = true;
            if (sim_thread_.joinable()) {
                sim_thread_.join();
            }

            if (!current_thread_status) {
                RCLCPP_INFO_STREAM(this->get_logger(), "Simulation thread stopped.");
            }
        }

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
        // Hold the thread where the simulation takes place
        std::thread sim_thread_;

        // Flag to signal stopping the thread
        std::atomic<bool> stop_thread_;

        std::string state_timer_key_;
        std::string state_pub_key_;

      private:
    };
} // namespace obelisk
