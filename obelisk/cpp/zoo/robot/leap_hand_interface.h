#include <cmath>
#include <iostream>

#include "dynamixel_sdk.h"
#include "obelisk_robot.h"
#include "obelisk_ros_utils.h"
#include "obelisk_sensor_msgs/msg/joint_encoders.h"

namespace obelisk {
    using leap_control_msg = obelisk_control_msgs::msg::PositionSetpoint;
    using leap_sensor_msg  = obelisk_sensor_msgs::msg::JointEncoders;

    class ObeliskLeapHand : public ObeliskRobot<leap_control_msg> {

      public:
        ObeliskLeapHand(const std::string& node_name, const std::string& state_timer_key = "timer_sensor",
                        const std::string& state_pub_key = "pub_sensor")
            : ObeliskRobot<leap_control_msg>(node_name),
              port_handler_(dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0")),
              packet_handler_(dynamixel::PacketHandler::getPacketHandler()),
              group_reader_(port_handler_, packet_handler_), group_writer_(port_handler_, packet_handler_),
              state_timer_key_(state_timer_key), state_pub_key_(state_pub_key) {
            this->RegisterObkTimer("timer_sensor_setting", state_timer_key_,
                                   std::bind(&ObeliskLeapHand::PublishState, this));
            this->template RegisterObkPublisher<leap_sensor_msg>("pub_sensor_setting", state_pub_key_);
            if (port_handler_->openPort()) {
                std::cout << "Opened port\n";
            } else {
                std::cerr << "Failed to open port\n";
            }
            if (port_handler_->setBaudRate(4000000)) {
                std::cout << "Set baud rate to 4000000\n";
            } else {
                std::cerr << "Failed to set baud rate\n";
            }
            for (int i = 1; i < N_MOTORS; i++) {
                int err = packet_handler_->write1ByteTxRx(port_handler_, i, 64, 1); // Enable torque
                err |= packet_handler_->write2ByteTxRx(port_handler_, i, 84, 500);  // kP
                err |= packet_handler_->write2ByteTxRx(port_handler_, i, 82, 10);   // kI
                err |= packet_handler_->write2ByteTxRx(port_handler_, i, 80, 50);   // kD
                if (err != 0) {
                    std::cerr << "Failed to initialize motor " << i << "\n";
                }
            }
            std::cout << "Initialized motors\n";
        }

        void ApplyControl(const leap_control_msg& control_msg) {
            for (int i = 1; i < N_MOTORS; i++) {
                uint32_t pos       = ObeliskLeapHand::RadiansToDxlPos(control_msg.u.at(i));
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

        void PublishState() {
            auto msg = leap_sensor_msg();
            for (int i = 1; i < N_MOTORS; i++) {
                group_reader_.addParam(i, PRESENT_POS_ADDR, PRESENT_POS_LENGTH);
            }
            if (this->group_reader_.txRxPacket() != COMM_SUCCESS) {
                std::cerr << "Failed to read motors\n";
            }
            for (int i = 0; i < N_MOTORS; i++) {
                int pos = group_reader_.getData(i, PRESENT_POS_ADDR, PRESENT_POS_LENGTH);
                msg.y.emplace_back(ObeliskLeapHand::DxlPosToRadians(pos));
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

        static constexpr int DXL_MAX_POS        = 4095;
        static constexpr int DXL_MIN_POS        = 0;
        static constexpr int N_MOTORS           = 16;

        static constexpr int GOAL_POS_ADDR      = 116;
        static constexpr int GOAL_POS_LENGTH    = 4;
        static constexpr int PRESENT_POS_ADDR   = 132;
        static constexpr int PRESENT_POS_LENGTH = 4;

        static inline int RadiansToDxlPos(const double& rads) {
            return (rads + M_PI) / (2 * M_PI) * (DXL_MAX_POS - DXL_MIN_POS);
        }
        static inline double DxlPosToRadians(const int& dxl_pos) {
            return double(dxl_pos) / (DXL_MAX_POS - DXL_MIN_POS) * 2 * M_PI - M_PI;
        }
    };
} // namespace obelisk