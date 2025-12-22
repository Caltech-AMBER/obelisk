#pragma once

#include "obelisk_robot.h"
#include "obelisk_sensor_msgs/msg/true_sim_state.hpp"

namespace obelisk {
    /**
     * @brief Abstract Obelisk simulated robot node.
     *
     * Simulated robots can be seen as a special case of the hardware robot, where all the sensors used to control the
     * robot are contained in the simulator and privileged information about the system can be published directly for
     * logging or debugging purposes. This privileged information is known as the TrueSimState of the simulator.
     *
     * Each ObeliskSimRobot is associated with a simulator. For instance, we currently support MuJoCo, but there is
     * nothing preventing the end user from implementing their own simulator of choice or us from implementing other
     * simulators.
     *
     */
    template <typename ControlMessageT> class ObeliskSimRobot : public ObeliskRobot<ControlMessageT> {
        using TrueSimState = obelisk_sensor_msgs::msg::TrueSimState;

      public:
        explicit ObeliskSimRobot(const std::string& name)
            : ObeliskRobot<ControlMessageT>(name) {

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
         * @brief Activates up the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn virtual on_activate(
            const rclcpp_lifecycle::State& prev_state) {
            this->ObeliskRobot<ControlMessageT>::on_activate(prev_state);

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief Deactivates up the node.
         *
         * @param prev_state the state of the ros node.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn virtual on_deactivate(
            const rclcpp_lifecycle::State& prev_state) {
            this->ObeliskRobot<ControlMessageT>::on_deactivate(prev_state);

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief Cleans up the node.
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
         * @brief Shutsdown the node the node.
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
        /**
         * @brief Ends the simulation thread
         *
         * Cleanly ends the simulation thread by switching the flag then joining the thread.
         * Logs the end of the thread.
         */
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

        /**
         * @brief Calls the RunSimulator() function
         *
         * This is here to avoid potential issues binding an abstract method.
         * This may be removed if these potential issues are determined to be non-issues.
         */
        void StartSim() { this->RunSimulator(); }

        // ---------- Member Variables --------- //
        // Hold the thread where the simulation takes place
        std::thread sim_thread_;

        // Flag to signal stopping the thread
        std::atomic<bool> stop_thread_;

      private:
    };
} // namespace obelisk
