#include "obelisk_node.h"

namespace obelisk {
    template <typename EstimatorMessageT> class ObeliskEstimator : public ObeliskNode {
      public:
        explicit ObeliskEstimator(const std::string& name) : ObeliskNode(name) {
            this->declare_parameter<std::string>("timer_est_config_str", "");
            this->declare_parameter<std::string>("pub_est_config_str", "");
            this->declare_parameter<std::vector<std::string>>("sub_sensor_config_strs", {""});
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& prev_state) {
            ObeliskNode::on_configure(prev_state);

            estimator_publisher_ =
                CreatePublisherFromConfigStr<EstimatorMessageT>(this->get_parameter("pub_est_config_str").as_string());

            estimator_timer_ = CreateWallTimerFromConfigStr(this->get_parameter("timer_est_config_str").as_string(),
                                                            std::bind(&ObeliskEstimator::ComputeStateEstimate, this));

            // The downstream user must create all their sensor subscribers

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief activates the node
         *
         * @param prev_state the state of the ros node
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& prev_state) {
            estimator_publisher_->on_activate();
            // TODO: Do anything with the timer?

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief deactivates the node
         *
         * @param prev_state the state of the ros node
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& prev_state) {
            estimator_publisher_->on_deactivate();
            // TODO: Do anything with the timer?

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief cleans up the node
         *
         * @param prev_state the state of the ros node
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State& prev_state) {
            // Release the shared pointers
            estimator_publisher_.reset();
            estimator_timer_.reset();

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        /**
         * @brief shutsdown the ros node
         *
         * @param prev_state the state of the ros node
         */
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State& prev_state) {
            // Release the shared pointers
            estimator_publisher_.reset();
            estimator_timer_.reset();

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

      protected:
        /**
         * @brief Abstract method to be implemented downstream. This is
         * automatically registered as the timer callback.
         */
        virtual EstimatorMessageT ComputeStateEstimate() = 0;

        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<EstimatorMessageT>> estimator_publisher_;

        // timer to activate ComputeControl
        rclcpp::TimerBase::SharedPtr estimator_timer_;

      private:
    };
} // namespace obelisk
