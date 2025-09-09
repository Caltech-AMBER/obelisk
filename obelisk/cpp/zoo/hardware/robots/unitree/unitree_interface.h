# pragma once
#include "obelisk_robot.h"
#include "obelisk_ros_utils.h"
#include "obelisk_sensor_msgs/msg/obk_joint_encoders.h"
#include "obelisk_sensor_msgs/msg/obk_imu.h"
#include "obelisk_control_msgs/msg/pd_feed_forward.h"
#include "obelisk_control_msgs/msg/execution_fsm.hpp"
#include "obelisk_control_msgs/msg/velocity_command.hpp"
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>


// DDS
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

// IDL
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

namespace obelisk {
    using unitree_control_msg = obelisk_control_msgs::msg::PDFeedForward;
    using unitree_fsm_msg = obelisk_control_msgs::msg::ExecutionFSM;
    using unitree_high_level_ctrl_msg = obelisk_control_msgs::msg::VelocityCommand;

    using namespace unitree::common;
    using namespace unitree::robot;

    /**
     * @brief Obelisk robot interface for the Unitree robots
     */
    class ObeliskUnitreeInterface : public ObeliskRobot<unitree_control_msg> {

    public:
        ObeliskUnitreeInterface(const std::string& node_name)
            : ObeliskRobot<unitree_control_msg>(node_name) {

            // Get network interface name as a parameter
            this->declare_parameter<std::string>("network_interface_name", "");
            network_interface_name_ = this->get_parameter("network_interface_name").as_string();
            RCLCPP_INFO_STREAM(this->get_logger(), "Network interface: " << network_interface_name_);

            // Set the Execution FSM into INIT
            exec_fsm_state_ = ExecFSMState::INIT;

            // Additional Publishers
            this->RegisterObkPublisher<obelisk_sensor_msgs::msg::ObkJointEncoders>("pub_sensor_setting", pub_joint_state_key_);
            this->RegisterObkPublisher<obelisk_sensor_msgs::msg::ObkImu>("pub_imu_setting", pub_imu_state_key_);

            // Optional low level logging
            this->declare_parameter<bool>("logging", false);
            logging_ = this->get_parameter("logging").as_bool();

            // Create logging directory if logging is enabled
            if (logging_) {
                auto now = std::chrono::system_clock::now();
                auto time_t = std::chrono::system_clock::to_time_t(now);
                std::stringstream ss;
                ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d_%H-%M-%S");
                
                std::filesystem::path unitree_logs_dir = "unitree_logs";
                std::filesystem::path session_dir = unitree_logs_dir / ss.str();
                
                std::filesystem::create_directories(session_dir);
                log_dir_path_ = session_dir.string();
                RCLCPP_INFO_STREAM(this->get_logger(), "Created logging directory: " << session_dir);
            }

            // Register Execution FSM Subscriber
            this->RegisterObkSubscription<unitree_fsm_msg>(
                "sub_fsm_setting", sub_fsm_key_, std::bind(&ObeliskUnitreeInterface::TransitionFSM, this, std::placeholders::_1));
            // Register High Level Control Subscriber
            this->RegisterObkSubscription<unitree_high_level_ctrl_msg>(
                "sub_high_level_ctrl_setting", sub_high_level_ctrl_key_, std::bind(&ObeliskUnitreeInterface::ApplyHighLevelControl, this, std::placeholders::_1));

            // ---------- Default PD gains ---------- //
            this->declare_parameter<std::vector<double>>("default_kp");
            this->declare_parameter<std::vector<double>>("default_kd");
            kp_ = this->get_parameter("default_kp").as_double_array();
            kd_ = this->get_parameter("default_kd").as_double_array();

            // TODO: User defined safety maybe
            
            ChannelFactory::Instance()->Init(0, network_interface_name_);

            // Try to shutdown motion control-related service
            mode_switch_manager_ = std::make_shared<b2::MotionSwitcherClient>();
            mode_switch_manager_->SetTimeout(5.0f);
            mode_switch_manager_->Init();
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn virtual on_activate(
            const rclcpp_lifecycle::State& prev_state) {
            this->ObeliskRobot::on_activate(prev_state);

            CreateUnitreeSubscribers();

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        enum class ExecFSMState : uint8_t {
            INIT = 0,
            UNITREE_HOME = 1,
            USER_POSE = 2,
            LOW_LEVEL_CTRL = 3,
            HIGH_LEVEL_CTRL = 4,
            DAMPING = 5
        };

        const std::unordered_map<ExecFSMState, std::string> TRANSITION_STRINGS = {
            {ExecFSMState::INIT, "INIT"},
            {ExecFSMState::UNITREE_HOME, "UNITREE_HOME"},
            {ExecFSMState::USER_POSE, "USER_POSE"},
            {ExecFSMState::LOW_LEVEL_CTRL, "LOW_LEVEL_CONTROL"},
            {ExecFSMState::HIGH_LEVEL_CTRL, "HIGH_LEVEL_CONTROL"},
            {ExecFSMState::DAMPING, "DAMPING"}
        };

        // Set of legal transitions for the execution FSM
        const std::unordered_map<ExecFSMState, std::vector<ExecFSMState>> TRANSITIONS = {
            {ExecFSMState::INIT, {ExecFSMState::UNITREE_HOME, ExecFSMState::DAMPING}},                                                                    // Init -> Home, Init -> Damp
            {ExecFSMState::UNITREE_HOME, {ExecFSMState::USER_POSE, ExecFSMState::LOW_LEVEL_CTRL, ExecFSMState::HIGH_LEVEL_CTRL, ExecFSMState::DAMPING}},  // Home -> Pose, Home -> Low,  Home -> High, Home -> Damp
            {ExecFSMState::USER_POSE, {ExecFSMState::UNITREE_HOME, ExecFSMState::LOW_LEVEL_CTRL, ExecFSMState::DAMPING}},                                 // Pose -> Home, Pose -> Low,  Pose -> Damp
            {ExecFSMState::LOW_LEVEL_CTRL, {ExecFSMState::UNITREE_HOME, ExecFSMState::USER_POSE, ExecFSMState::DAMPING}},                                 // Low  -> Home, Low  -> Pose, Low  -> Damp
            {ExecFSMState::HIGH_LEVEL_CTRL, {ExecFSMState::UNITREE_HOME, ExecFSMState::DAMPING}},                                                         // High -> Home, High -> Damp
            {ExecFSMState::DAMPING, {ExecFSMState::UNITREE_HOME, ExecFSMState::USER_POSE}}                                                                // Damp -> Home, Damp -> Pose
        };

        // Set of transitions on which motion control must be released
        const std::vector<ExecFSMState> RELEASE_MC_STATES = {
            ExecFSMState::USER_POSE, ExecFSMState::LOW_LEVEL_CTRL, ExecFSMState::DAMPING
        };

        // Set of transitions on which motion control must be engaged
        const std::vector<ExecFSMState> ENGAGE_MC_STATES = {
            ExecFSMState::UNITREE_HOME, ExecFSMState::HIGH_LEVEL_CTRL
        };

    protected:
        virtual void CreateUnitreeSubscribers() = 0;
        virtual void CreateUnitreePublishers() = 0;
        virtual void ApplyHighLevelControl(const unitree_high_level_ctrl_msg& msg) = 0;
        virtual bool CheckDampingToUnitreeHomeTransition() = 0;
        virtual void TransitionToUnitreeHome() = 0;
        virtual void TransitionToUserPose() = 0;

        void TransitionFSM(const unitree_fsm_msg& msg) {
            // Extract commanded FSM state from the message
            ExecFSMState cmd_exec_fsm_state = static_cast<ExecFSMState>(msg.cmd_exec_fsm_state);
            
            // Check if transition is legal
            if (ContainsTransition(TRANSITIONS, exec_fsm_state_, cmd_exec_fsm_state)) {
                // Check if robot is in OK state to transition from Damping to Home
                if (exec_fsm_state_ == ExecFSMState::DAMPING && cmd_exec_fsm_state == ExecFSMState::UNITREE_HOME && !CheckDampingToUnitreeHomeTransition()) {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "ROBOT NOT IN VALID STATE TO TRANSITION EXECUTION FSM FROM DAMPING TO UNITREE_HOME!");
                    return;
                }

                // Under some transitions, need to release the motion control
                if (std::find(RELEASE_MC_STATES.begin(), RELEASE_MC_STATES.end(), cmd_exec_fsm_state) != RELEASE_MC_STATES.end()) {
                    bool success = ReleaseMotionControl();
                    if (!success) {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "RELEASING MOTION CONTROL FAILED!");
                        return;
                    }
                    RCLCPP_INFO_STREAM(this->get_logger(), "RELEASED MOTION CONTROL!");
                }

                // Under some transitions, need to re-engage the motion control
                if (std::find(ENGAGE_MC_STATES.begin(), ENGAGE_MC_STATES.end(), cmd_exec_fsm_state) != ENGAGE_MC_STATES.end()) {
                    bool success = EngageMotionControl();
                    if (!success) {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "ENGAGING MOTION CONTROL FAILED!");
                        return;
                    }
                    RCLCPP_INFO_STREAM(this->get_logger(), "ENGAGED MOTION CONTROL!");
                }
  
                // Transition the FSM
                exec_fsm_state_ = cmd_exec_fsm_state;
                RCLCPP_INFO_STREAM(this->get_logger(), "EXECUTION FSM STATE TRANSITIONED TO " << TRANSITION_STRINGS.at(exec_fsm_state_));
                
                if (exec_fsm_state_ == ExecFSMState::UNITREE_HOME) {
                    TransitionToUnitreeHome();
                } else if (exec_fsm_state_ == ExecFSMState::USER_POSE) {
                    TransitionToUserPose();
                }
            } else {
                if (exec_fsm_state_ == cmd_exec_fsm_state) {
                    RCLCPP_INFO_STREAM(this->get_logger(), "EXECUTION FSM AREADY IN COMMANDED STATE " <<  TRANSITION_STRINGS.at(exec_fsm_state_));
                } else {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "EXECUTION FSM COMMAND INVALID: " <<  TRANSITION_STRINGS.at(exec_fsm_state_) << " -> " <<  TRANSITION_STRINGS.at(cmd_exec_fsm_state));
                }
            }
        }


        void VerifyParameters() {
            // Verify default gains
            if (kp_.empty()) {
                for (size_t i = 0; i < num_motors_; i++) {
                    kp_.push_back(0);
                }
            }

            if (kd_.empty()) {
                for (size_t i = 0; i < num_motors_; i++) {
                    kd_.push_back(0);
                }
            }

            if (kp_.size() != num_motors_ || kd_.size() != num_motors_) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Expected size: " << num_motors_);
                RCLCPP_ERROR_STREAM(this->get_logger(), "default kp size: " << kp_.size());
                RCLCPP_ERROR_STREAM(this->get_logger(), "default kd size: " << kd_.size());
                throw std::runtime_error("[UnitreeRobotInterface] default gains have incorrect sizes!");
            }
        }

        inline uint32_t Crc32Core(uint32_t *ptr, uint32_t len) {
            uint32_t xbit = 0;
            uint32_t data = 0;
            uint32_t CRC32 = 0xFFFFFFFF;
            const uint32_t dwPolynomial = 0x04c11db7;

            for (uint32_t i = 0; i < len; i++) {
                xbit = 1 << 31;
                data = ptr[i];
                for (uint32_t bits = 0; bits < 32; bits++) {
                if (CRC32 & 0x80000000) {
                    CRC32 <<= 1;
                    CRC32 ^= dwPolynomial;
                } else
                    CRC32 <<= 1;
                if (data & xbit) CRC32 ^= dwPolynomial;

                xbit >>= 1;
                }
            }

            return CRC32;
        };

        bool ContainsTransition(std::unordered_map<ExecFSMState, std::vector<ExecFSMState>> transitions,
                                ExecFSMState state,
                                ExecFSMState next_state) {
            return std::find(transitions[state].begin(), transitions[state].end(), next_state) != transitions[state].end();
        }

        // bool ReleaseMotionControl() {
        //     int32_t ret = mode_switch_manager_->ReleaseMode();
        //     return ret == 0;
        // }

        bool ReleaseMotionControl() {
            std::string robot_form, motion_mode;
            int ret = mode_switch_manager_->CheckMode(robot_form, motion_mode);
            if (ret == 0) {
                RCLCPP_INFO_STREAM(this->get_logger(), "[UnitreeInterface] Check mode succeeded.");
            } else {
                RCLCPP_ERROR_STREAM(this->get_logger(), "[UnitreeInterface] Check mode failed. Error code: " << ret );
                return false;
            }

            if (!motion_mode.empty()) {
                return mode_switch_manager_->ReleaseMode() == 0;
            }
            return true;
        }

        // bool EngageMotionControl() {
        //     // TODO: What goes here??
        //     int32_t ret = mode_switch_manager_->SelectMode("normal");  // Can be "normal", "ai", or "advanced" ?? From reading example code.
        //     return ret == 0;
        // }

        bool EngageMotionControl() {
            std::string robot_form, motion_mode;
            mode_switch_manager_->CheckMode(robot_form, motion_mode);
            if (motion_mode.empty()) {
                return mode_switch_manager_->SelectMode("normal") == 0;
            }
            return true;
        }

        // Execution FSM state variable
        ExecFSMState exec_fsm_state_;

        // ---------- Topics ---------- //
        // TODO: Verify these are the same of the G1 and the Go2
        std::string CMD_TOPIC_;
        std::string STATE_TOPIC_;

        std::string network_interface_name_;
        std::shared_ptr<b2::MotionSwitcherClient> mode_switch_manager_;

        size_t num_motors_;
        std::vector<std::string> joint_names_;

        // ---------- Gains ---------- //
        std::vector<double> kp_;
        std::vector<double> kd_;

        // Keys
        const std::string sub_fsm_key_ = "sub_exec_fsm_key";
        const std::string sub_high_level_ctrl_key_ = "sub_high_level_ctrl_key";
        const std::string pub_joint_state_key_ = "joint_state_pub";
        const std::string pub_imu_state_key_ = "imu_state_pub";

        // Logging
        bool logging_;
        std::string log_dir_path_;
    
    private:
    };
} // namespace obelisk