#include <cmath>
#include <iostream>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "obelisk_robot.h"
#include "obelisk_ros_utils.h"
#include "obelisk_sensor_msgs/msg/obk_joint_encoders.h"

namespace obelisk {
    using leap_control_msg = obelisk_control_msgs::msg::PositionSetpoint;
    using leap_sensor_msg  = obelisk_sensor_msgs::msg::ObkJointEncoders;

    /**
     * @brief Obelisk robot interface for the Leap hand hardware
     */
    class ObeliskLeapHand : public ObeliskRobot<leap_control_msg> {

      public:
        /**
         * @brief Construct a new Obelisk Leap Hand object
         *
         * @param node_name Name of the node
         * @param state_timer_key Key for the state timer
         * @param state_pub_key Key for the state publisher
         */
        ObeliskLeapHand(const std::string& node_name, const std::string& state_timer_key = "timer_sensor",
                        const std::string& state_pub_key = "pub_sensor")
            : ObeliskRobot<leap_control_msg>(node_name),
              port_handler_(dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0")),
              packet_handler_(dynamixel::PacketHandler::getPacketHandler()),
              group_reader_(port_handler_, packet_handler_), group_writer_(port_handler_, packet_handler_),
              state_timer_key_(state_timer_key), state_pub_key_(state_pub_key) {

            // exposing gains as ros params
            this->declare_parameter("KP", 150);
            this->declare_parameter("KI", 0);
            this->declare_parameter("KD", 50);

            // timer/pub pair for the leap hand state
            this->RegisterObkTimer("timer_sensor_setting", state_timer_key_,
                                   std::bind(&ObeliskLeapHand::PublishState, this));
            this->template RegisterObkPublisher<leap_sensor_msg>("pub_sensor_setting", state_pub_key_);

            // setting up ports and initial values
            if (port_handler_->openPort()) {
                std::cout << "Opened port\n";
            } else {
                std::cerr << "Failed to open port\n";
            }

            if (port_handler_->setBaudRate(BAUDRATE)) {
                std::cout << "Set baud rate to" << BAUDRATE << "\n";
            } else {
                std::cerr << "Failed to set baud rate\n";
            }
        }

        /**
         * @brief Apply a control message to the Leap hand
         *
         * @param control_msg Control message to apply
         */
        void ApplyControl(const leap_control_msg& control_msg) {
            for (int i = 0; i < N_MOTORS; i++) {
                uint32_t pos       = ObeliskLeapHand::RadiansToDxlPos(control_msg.q_des.at(i));
                uint8_t pos_arr[4] = {
                    static_cast<uint8_t>(pos),
                    static_cast<uint8_t>(pos >> 8),
                    static_cast<uint8_t>(pos >> 16),
                    static_cast<uint8_t>(pos >> 24),
                };
                if (!group_writer_.addParam(i, GOAL_POS_ADDR, GOAL_POS_LENGTH, pos_arr)) {
                    std::cerr << "Motor " << i << " failed to add param\n";
                }
            }
            group_writer_.txPacket();
            group_writer_.clearParam();
        }

        /**
         * @brief Publish the current state of the Leap hand. This function expects to be called
         * by a timer callback.
         */
        void PublishState() {
            static int repeated_errs        = 0;
            const static int MAX_REPEAT_ERR = 10; // we are reading at a high rate, so sometimes we will get errors
            auto msg                        = leap_sensor_msg();
            for (int i = 0; i < N_MOTORS; i++) {
                group_reader_.addParam(i, PRESENT_POS_ADDR, PRESENT_POS_LENGTH);
            }
            if (group_reader_.txRxPacket() != COMM_SUCCESS) {
                repeated_errs++;
                if (repeated_errs > MAX_REPEAT_ERR) {
                    std::cerr << "Failed to read motors\n";
                }
                return;
            }
            repeated_errs = 0;
            for (int i = 0; i < N_MOTORS; i++) {
                int pos = group_reader_.getData(i, PRESENT_POS_ADDR, PRESENT_POS_LENGTH);
                msg.joint_pos.emplace_back(ObeliskLeapHand::DxlPosToRadians(pos));
            }
            this->GetPublisher<leap_sensor_msg>(state_pub_key_)->publish(msg);
        }

      private:
        dynamixel::PortHandler* port_handler_;
        dynamixel::PacketHandler* packet_handler_;
        dynamixel::GroupBulkRead group_reader_;
        dynamixel::GroupBulkWrite group_writer_;

        const std::string state_timer_key_;
        const std::string state_pub_key_;

        static constexpr int BAUDRATE = 4000000;

        static constexpr int DXL_MAX_POS = 4095;
        static constexpr int DXL_MIN_POS = 0;
        static constexpr int N_MOTORS    = 16;

        static constexpr int GOAL_POS_ADDR      = 116;
        static constexpr int GOAL_POS_LENGTH    = 4;
        static constexpr int PRESENT_POS_ADDR   = 132;
        static constexpr int PRESENT_POS_LENGTH = 4;

        static constexpr int TORQUE_ADDR       = 64;
        static constexpr int TORQUE_ENABLE_VAL = 1;
        static constexpr int KP_ADDR           = 84;
        static constexpr int KI_ADDR           = 82;
        static constexpr int KD_ADDR           = 80;

        double KP_;
        double KI_;
        double KD_;

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& state) {
            ObeliskRobot::on_configure(state);

            // setting gains and activating motors
            KP_ = this->get_parameter("KP").as_int();
            KI_ = this->get_parameter("KI").as_int();
            KD_ = this->get_parameter("KD").as_int();

            for (int i = 0; i < N_MOTORS; i++) {
                int err =
                    packet_handler_->write1ByteTxRx(port_handler_, i, TORQUE_ADDR, TORQUE_ENABLE_VAL); // Enable torque
                err |= packet_handler_->write2ByteTxRx(port_handler_, i, KP_ADDR, KP_);                // KP
                err |= packet_handler_->write2ByteTxRx(port_handler_, i, KI_ADDR, KI_);                // KI
                err |= packet_handler_->write2ByteTxRx(port_handler_, i, KD_ADDR, KD_);                // KD
                if (err != 0) {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to initialize motor " << i);
                }
            }
            RCLCPP_INFO_STREAM(this->get_logger(), "Initialized motors!");

            // homing the LEAP hand
            leap_control_msg home_msg  = leap_control_msg();
            std::vector<double> q_home = {0.5, -0.75, 0.75, 0.25, 0.5,  0.0, 0.75, 0.25,
                                          0.5, 0.75,  0.75, 0.25, 0.65, 0.9, 0.75, 0.6};
            home_msg.q_des             = q_home;
            ApplyControl(home_msg);

            RCLCPP_INFO_STREAM(this->get_logger(), "Configured Leap Hand!");
            return CallbackReturn::SUCCESS;
        }

        static inline int RadiansToDxlPos(const double& rads) {
            return (rads + M_PI) / (2 * M_PI) * (DXL_MAX_POS - DXL_MIN_POS);
        }
        static inline double DxlPosToRadians(const int& dxl_pos) {
            return double(dxl_pos) / (DXL_MAX_POS - DXL_MIN_POS) * 2 * M_PI - M_PI;
        }
    };
} // namespace obelisk
