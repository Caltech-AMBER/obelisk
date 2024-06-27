#include "obelisk_robot.h"
#include "obelisk_sensor_msgs/msg/true_sim_state.hpp"

namespace obelisk {
    template <typename ControlMessageT> class ObeliskSimRobot : public ObeliskRobot<ControlMessageT> {
        using TrueSimState = obelisk_sensor_msgs::msg::TrueSimState;

      public:
        explicit ObeliskSimRobot(const std::string& name) : ObeliskRobot<ControlMessageT>(name) {
            this->template declare_parameter<int>("n_u", -1);
            this->template declare_parameter<std::string>("timer_true_sim_state_setting", "");
            this->template declare_parameter<std::string>("pub_true_sim_state_setting", "");
            stop_thread_ = false;
        }

        /**
         * @brief Configures all the required ROS components. Specifcially this
         * registers the true_sim_state_publisher_ if a configuration is passed. Also makes a call to ObeliskNode on
         * configure to parse and create the callback group map.
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& prev_state) {
            ObeliskNode::on_configure(prev_state);

            n_u_ = this->get_parameter("n_u").as_int();
            if (n_u_ <= 0) {
                throw std::runtime_error("Invalid control input dimension - must be greater 0!");
            }

            const std::string timer_settings = this->get_parameter("timer_true_sim_state_setting").as_string();
            const std::string pub_settings   = this->get_parameter("pub_true_sim_state_setting").as_string();

            if (timer_settings != "" && pub_settings != "") {
                true_sim_state_timer_ = this->CreateWallTimerFromConfigStr(
                    timer_settings, std::bind(&ObeliskSimRobot::PublishTrueSimState, this));
                true_sim_state_publisher_ = this->template CreatePublisherFromConfigStr<TrueSimState>(pub_settings);
            } else {
                RCLCPP_WARN_STREAM(this->get_logger(), "No true simulation state timer/publisher configured.");
                true_sim_state_timer_     = nullptr;
                true_sim_state_publisher_ = nullptr;
            }

            shared_data_.resize(n_u_);
            stop_thread_ = false;

            // Start the simulator
            sim_thread_ = std::thread(&ObeliskSimRobot::RunSimulator, this);
            RCLCPP_INFO_STREAM(this->get_logger(), "Simulation thread started.");

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

            // Reset data
            n_u_ = -1;

            // Release the shared pointers
            true_sim_state_publisher_.reset();

            // TODO: Cleanup the sim thread
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
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State& prev_state) {
            ObeliskNode::on_shutdown(prev_state);

            return on_cleanup(prev_state);
        }

      protected:
        /**
         * @brief Safely set the shared data.
         *
         * @param data the data to put into shared_data_
         */
        void SetSharedData(const std::vector<double>& data) {
            std::lock_guard<std::mutex> lock(shared_data_mut_);

            // Copy data in
            shared_data_ = data;
        }

        /**
         * @brief Safely retrieves the contents of shared data.
         *
         * @param data [output] where the shared data will be copied
         */
        void GetSharedData(std::vector<double>& data) {
            std::lock_guard<std::mutex> lock(shared_data_mut_);

            // Copy data out
            data = shared_data_;
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

        // ---------- Member Variables --------- //
        // Number of control inputs
        int n_u_;

        // Publisher for the true sim state
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<TrueSimState>> true_sim_state_publisher_;

        // Timer to publish the true sim state
        rclcpp::TimerBase::SharedPtr true_sim_state_timer_;

        // Shared data between the main thread and the sim thread
        std::vector<double> shared_data_;

        // Hold the thread where the simulation takes place
        std::thread sim_thread_;

        // Mutex to manage access to the shared data
        std::mutex shared_data_mut_;

        // Flag to signal stopping the thread
        std::atomic<bool> stop_thread_;

      private:
    };
} // namespace obelisk
