#pragma once

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "obelisk_robot.h"
#include "obelisk_ros_utils.h"
#include "obelisk_sensor_msgs/msg/obk_joint_encoders.hpp"
#include "obelisk_control_msgs/msg/execution_fsm.hpp"
#include "obelisk_control_msgs/msg/pd_feed_forward.hpp"
#include "obelisk_control_msgs/msg/velocity_command.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "themis_fsm.h"

namespace obelisk {

    using themis_control_msg  = obelisk_control_msgs::msg::PDFeedForward;
    using themis_fsm_msg      = obelisk_control_msgs::msg::ExecutionFSM;
    using themis_vel_cmd_msg  = obelisk_control_msgs::msg::VelocityCommand;

    /**
     * @brief Obelisk robot interface for the Westwood Robotics THEMIS humanoid.
     *
     * Communicates with the on-robot Python bridge
     * (obelisk_westwood_cpp/server/themis_obk_server.py) over UDP. The C++ side
     * never links AOS; the server translates UDP messages into AOS API calls.
     *
     * Three control modes, gated by the server's --mode flag and the FSM state:
     *   - USER_CTRL:      direct joint control via PDFeedForward (server runs in --mode user-ctrl)
     *   - HIGH_LEVEL_VEL: locomotion velocity commands via VelocityCommand (server in --mode high-level)
     *   - USER_POSE / DAMPING / ESTOP / INIT: see ThemisFSMState
     *
     * Wire protocol — see themis_obk_server.py header for the full byte layout.
     */
    class ThemisInterface : public ObeliskRobot<themis_control_msg> {
      public:
        // ─────────────────────────── wire protocol ──────────────────────────
        static constexpr uint8_t MSG_FULL_STATE_REQUEST  = 0x05;
        static constexpr uint8_t MSG_FULL_STATE_RESPONSE = 0x06;
        static constexpr uint8_t MSG_FSM_TRANSITION      = 0x07;
        static constexpr uint8_t MSG_BASE_VEL_CMD        = 0x14;
        static constexpr uint8_t MSG_BASE_ORIENT_CMD     = 0x15;
        static constexpr uint8_t MSG_FULL_BODY_CMD       = 0x16;
        static constexpr uint8_t MSG_COM_POS_CMD         = 0x18;
        static constexpr uint8_t MSG_HEARTBEAT           = 0x20;
        static constexpr uint8_t MSG_ACK                 = 0xFE;

        static constexpr size_t NUM_JOINTS         = 28;
        static constexpr size_t STATE_NUM_DOUBLES  = 100;  // q+dq+u+accel+gyro+R+ts
        static constexpr size_t CMD_NUM_DOUBLES    = 140;  // q+dq+u+kp+kd

        /// Canonical joint order: [right_leg(6), left_leg(6), right_arm(7), left_arm(7), head(2)].
        /// Must match themis_obk_server.py's CHAINS mapping.
        static const std::array<std::string, NUM_JOINTS>& ThemisJointNames() {
            static const std::array<std::string, NUM_JOINTS> NAMES = {
                "HIP_YAW_R", "HIP_ROLL_R", "HIP_PITCH_R", "KNEE_PITCH_R", "ANKLE_PITCH_R", "ANKLE_ROLL_R",
                "HIP_YAW_L", "HIP_ROLL_L", "HIP_PITCH_L", "KNEE_PITCH_L", "ANKLE_PITCH_L", "ANKLE_ROLL_L",
                "SHOULDER_PITCH_R", "SHOULDER_ROLL_R", "SHOULDER_YAW_R", "ELBOW_PITCH_R", "ELBOW_YAW_R",
                "WRIST_PITCH_R", "WRIST_YAW_R",
                "SHOULDER_PITCH_L", "SHOULDER_ROLL_L", "SHOULDER_YAW_L", "ELBOW_PITCH_L", "ELBOW_YAW_L",
                "WRIST_PITCH_L", "WRIST_YAW_L",
                "HEAD_YAW", "HEAD_PITCH",
            };
            return NAMES;
        }

        explicit ThemisInterface(const std::string& node_name) : ObeliskRobot<themis_control_msg>(node_name) {
            // ROS parameter declarations only; sockets + threads come up in on_configure / on_activate.
            this->declare_parameter<std::string>("target_ip", "192.168.0.11");
            this->declare_parameter<int>("udp_port", 9870);
            this->declare_parameter<double>("feedback_rate_hz", 100.0);
            this->declare_parameter<double>("heartbeat_period_s", 2.0);
            this->declare_parameter<double>("user_pose_transition_duration", 3.0);
            this->declare_parameter<bool>("enable_base_orient_cmd", false);
            this->declare_parameter<bool>("enable_com_pos_cmd", false);

            // Per-joint gains + home pose. Sizes verified in VerifyParameters().
            this->declare_parameter<std::vector<double>>("default_kp",         std::vector<double>{});
            this->declare_parameter<std::vector<double>>("default_kd",         std::vector<double>{});
            this->declare_parameter<std::vector<double>>("default_kd_damping", std::vector<double>{});
            this->declare_parameter<std::vector<double>>("user_pose",          std::vector<double>{});

            // Joint name override; defaults to the canonical THEMIS_JOINT_NAMES.
            std::vector<std::string> default_names(ThemisJointNames().begin(), ThemisJointNames().end());
            this->declare_parameter<std::vector<std::string>>("default_joint_names", default_names);

            // Obelisk components — the launch layer will fill these in from obelisk_settings.
            this->RegisterObkPublisher<obelisk_sensor_msgs::msg::ObkJointEncoders>(pub_joint_state_key_);
            this->RegisterObkPublisher<sensor_msgs::msg::Imu>(pub_imu_state_key_);
            this->RegisterObkTimer(timer_sensor_key_,
                                   std::bind(&ThemisInterface::PublishLatestState, this));

            this->RegisterObkSubscription<themis_fsm_msg>(
                sub_fsm_key_,
                std::bind(&ThemisInterface::TransitionFSM, this, std::placeholders::_1));
            this->RegisterObkSubscription<themis_vel_cmd_msg>(
                sub_vel_cmd_key_,
                std::bind(&ThemisInterface::ApplyVelCmd, this, std::placeholders::_1));
        }

        ~ThemisInterface() override {
            running_ = false;
            if (fb_thread_.joinable()) fb_thread_.join();
            if (hb_thread_.joinable()) hb_thread_.join();
            CloseSocket();
        }

        // ─────────────────────────── lifecycle ──────────────────────────────
        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State& prev_state) override {
            ObeliskRobot::on_configure(prev_state);

            target_ip_                    = this->get_parameter("target_ip").as_string();
            udp_port_                     = static_cast<uint16_t>(this->get_parameter("udp_port").as_int());
            feedback_rate_hz_             = this->get_parameter("feedback_rate_hz").as_double();
            heartbeat_period_s_           = this->get_parameter("heartbeat_period_s").as_double();
            user_pose_transition_duration_ = this->get_parameter("user_pose_transition_duration").as_double();
            enable_base_orient_cmd_       = this->get_parameter("enable_base_orient_cmd").as_bool();
            enable_com_pos_cmd_           = this->get_parameter("enable_com_pos_cmd").as_bool();

            kp_         = this->get_parameter("default_kp").as_double_array();
            kd_         = this->get_parameter("default_kd").as_double_array();
            kd_damping_ = this->get_parameter("default_kd_damping").as_double_array();
            user_pose_  = this->get_parameter("user_pose").as_double_array();
            default_joint_names_ = this->get_parameter("default_joint_names").as_string_array();

            VerifyParameters();
            BuildJointNameIndex();
            OpenSocket();

            fsm_state_ = ThemisFSMState::INIT;
            RCLCPP_INFO_STREAM(this->get_logger(), "ThemisInterface configured for "
                                                       << target_ip_ << ":" << udp_port_);
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State& prev_state) override {
            ObeliskRobot::on_activate(prev_state);

            running_ = true;
            fb_thread_ = std::thread(&ThemisInterface::FeedbackLoop, this);
            hb_thread_ = std::thread(&ThemisInterface::HeartbeatLoop, this);

            RCLCPP_INFO_STREAM(this->get_logger(), "ThemisInterface activated. Feedback + heartbeat threads up.");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State& prev_state) override {
            running_ = false;
            if (fb_thread_.joinable()) fb_thread_.join();
            if (hb_thread_.joinable()) hb_thread_.join();
            ObeliskRobot::on_deactivate(prev_state);
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State& prev_state) override {
            CloseSocket();
            ObeliskRobot::on_cleanup(prev_state);
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        // ─────────────────────────── control path ───────────────────────────
        /**
         * @brief Handle an incoming PDFeedForward control message.
         *
         * Routed by FSM state. USER_CTRL forwards as-is. USER_POSE overrides
         * with a cosine ramp from the snapshot pose. DAMPING / ESTOP / INIT
         * drop — the server owns those states (wbc_api.damping_motors /
         * disable_motors are called once on FSM transition).
         */
        void ApplyControl(const themis_control_msg& msg) override {
            std::array<double, NUM_JOINTS> q{}, dq{}, u{}, kp{}, kd{};

            ThemisFSMState state = fsm_state_.load();
            if (state == ThemisFSMState::USER_CTRL) {
                if (!PermuteIntoCanonicalOrder(msg, q, dq, u, kp, kd)) return;
                SendFullBodyCmd(q, dq, u, kp, kd);
            } else if (state == ThemisFSMState::USER_POSE) {
                ComputeUserPoseRamp(q, kp);
                std::fill(kd.begin(), kd.end(), 0.0);
                for (size_t i = 0; i < NUM_JOINTS; ++i) kd[i] = kd_[i];
                SendFullBodyCmd(q, dq, u, kp, kd);
            }
            // INIT / HIGH_LEVEL_VEL / DAMPING / ESTOP: drop silently.
        }

        /**
         * @brief Handle a VelocityCommand. Only acts in HIGH_LEVEL_VEL.
         */
        void ApplyVelCmd(const themis_vel_cmd_msg& msg) {
            if (fsm_state_.load() != ThemisFSMState::HIGH_LEVEL_VEL) return;
            SendVec3(MSG_BASE_VEL_CMD, msg.v_x, msg.v_y, msg.w_z);
        }

      protected:
        // ─────────────────────────── FSM ────────────────────────────────────
        void TransitionFSM(const themis_fsm_msg& msg) {
            const auto& transitions = ThemisFSMTransitions();
            const auto& names       = ThemisFSMNames();

            ThemisFSMState cur = fsm_state_.load();
            ThemisFSMState cmd;
            try {
                cmd = static_cast<ThemisFSMState>(msg.cmd_exec_fsm_state);
                (void)names.at(cmd);  // throws if unknown
            } catch (const std::out_of_range&) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Unknown FSM state: "
                                                           << static_cast<int>(msg.cmd_exec_fsm_state));
                return;
            }

            if (cur == ThemisFSMState::ESTOP) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "ESTOP is sticky; rejecting transition to " << names.at(cmd));
                return;
            }

            const auto& legal = transitions.at(cur);
            if (std::find(legal.begin(), legal.end(), cmd) == legal.end()) {
                if (cur == cmd) {
                    RCLCPP_INFO_STREAM(this->get_logger(), "Already in state " << names.at(cur));
                } else {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Illegal FSM transition: "
                                                                << names.at(cur) << " -> " << names.at(cmd));
                }
                return;
            }

            // Client-side state entry. Server-side entry actions ride the wire FSM_TRANSITION message.
            if (cmd == ThemisFSMState::USER_POSE) {
                SnapshotCurrentQAsRampStart();
                user_pose_transition_start_ = std::chrono::steady_clock::now();
            }

            SendFsmTransition(cmd);
            fsm_state_.store(cmd);
            RCLCPP_INFO_STREAM(this->get_logger(), "FSM " << names.at(cur) << " -> " << names.at(cmd));
        }

        // ─────────────────────────── UDP wire ───────────────────────────────
        void SendFullBodyCmd(const std::array<double, NUM_JOINTS>& q,
                             const std::array<double, NUM_JOINTS>& dq,
                             const std::array<double, NUM_JOINTS>& u,
                             const std::array<double, NUM_JOINTS>& kp,
                             const std::array<double, NUM_JOINTS>& kd) {
            std::vector<uint8_t> pkt;
            pkt.reserve(1 + CMD_NUM_DOUBLES * sizeof(double));
            pkt.push_back(MSG_FULL_BODY_CMD);
            AppendDoubles(pkt, q.data(),  NUM_JOINTS);
            AppendDoubles(pkt, dq.data(), NUM_JOINTS);
            AppendDoubles(pkt, u.data(),  NUM_JOINTS);
            AppendDoubles(pkt, kp.data(), NUM_JOINTS);
            AppendDoubles(pkt, kd.data(), NUM_JOINTS);
            SendDatagram(pkt);
        }

        void SendVec3(uint8_t msg_type, double a, double b, double c) {
            std::vector<uint8_t> pkt;
            pkt.reserve(1 + 3 * sizeof(double));
            pkt.push_back(msg_type);
            double abc[3] = {a, b, c};
            AppendDoubles(pkt, abc, 3);
            SendDatagram(pkt);
        }

        void SendFsmTransition(ThemisFSMState s) {
            uint8_t pkt[2] = {MSG_FSM_TRANSITION, static_cast<uint8_t>(s)};
            SendDatagram(pkt, sizeof(pkt));
        }

        void FeedbackLoop() {
            const auto dt = std::chrono::microseconds(
                static_cast<int>(1e6 / std::max(1.0, feedback_rate_hz_)));

            int sock = socket(AF_INET, SOCK_DGRAM, 0);
            if (sock < 0) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "FeedbackLoop: socket() failed: " << strerror(errno));
                return;
            }
            timeval tv{};
            tv.tv_sec  = 0;
            tv.tv_usec = 300'000;
            ::setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

            while (running_.load()) {
                // Request.
                uint8_t req = MSG_FULL_STATE_REQUEST;
                ::sendto(sock, &req, 1, 0,
                         reinterpret_cast<sockaddr*>(&server_addr_), sizeof(server_addr_));

                // Receive.
                uint8_t buf[2048];
                ssize_t n = ::recvfrom(sock, buf, sizeof(buf), 0, nullptr, nullptr);
                if (n > 1 && buf[0] == MSG_FULL_STATE_RESPONSE) {
                    ParseFullState(buf + 1, static_cast<size_t>(n - 1));
                }
                std::this_thread::sleep_for(dt);
            }
            ::close(sock);
        }

        void HeartbeatLoop() {
            const auto period = std::chrono::milliseconds(
                static_cast<int>(1000.0 * heartbeat_period_s_));
            while (running_.load()) {
                uint8_t hb = MSG_HEARTBEAT;
                SendDatagram(&hb, 1);
                std::this_thread::sleep_for(period);
            }
        }

        void PublishLatestState() {
            LatestState snap;
            {
                std::lock_guard<std::mutex> lk(state_mtx_);
                snap = latest_state_;
            }
            if (!snap.valid) return;

            // ObkJointEncoders
            obelisk_sensor_msgs::msg::ObkJointEncoders js;
            js.header.stamp = this->now();
            js.joint_names.assign(ThemisJointNames().begin(), ThemisJointNames().end());
            js.joint_pos.assign(snap.q.begin(),  snap.q.end());
            js.joint_vel.assign(snap.dq.begin(), snap.dq.end());
            // Note: AOS v0.2.4 does not expose motor temperatures or voltages.
            // motor_surface_temps / motor_winding_temps are intentionally left empty.
            this->GetPublisher<obelisk_sensor_msgs::msg::ObkJointEncoders>(pub_joint_state_key_)->publish(js);

            // Imu — rotation matrix → quaternion via Shepperd's method.
            sensor_msgs::msg::Imu imu;
            imu.header.stamp = this->now();
            imu.linear_acceleration.x = snap.imu_accel[0];
            imu.linear_acceleration.y = snap.imu_accel[1];
            imu.linear_acceleration.z = snap.imu_accel[2];
            imu.angular_velocity.x = snap.imu_gyro[0];
            imu.angular_velocity.y = snap.imu_gyro[1];
            imu.angular_velocity.z = snap.imu_gyro[2];
            double qw, qx, qy, qz;
            RotMat3x3ToQuaternion(snap.R, qw, qx, qy, qz);
            imu.orientation.w = qw;
            imu.orientation.x = qx;
            imu.orientation.y = qy;
            imu.orientation.z = qz;
            this->GetPublisher<sensor_msgs::msg::Imu>(pub_imu_state_key_)->publish(imu);
        }

        // ─────────────────────────── helpers ────────────────────────────────
        bool PermuteIntoCanonicalOrder(const themis_control_msg& msg,
                                       std::array<double, NUM_JOINTS>& q,
                                       std::array<double, NUM_JOINTS>& dq,
                                       std::array<double, NUM_JOINTS>& u,
                                       std::array<double, NUM_JOINTS>& kp,
                                       std::array<double, NUM_JOINTS>& kd) {
            if (msg.joint_names.size() != NUM_JOINTS) {
                RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "PDFeedForward joint_names size " << msg.joint_names.size() << " != " << NUM_JOINTS);
                return false;
            }
            if (msg.pos_target.size() != NUM_JOINTS) return false;

            const bool has_kp = (msg.kp.size() == NUM_JOINTS);
            const bool has_kd = (msg.kd.size() == NUM_JOINTS);
            const bool has_vel = (msg.vel_target.size() == NUM_JOINTS);
            const bool has_ff  = (msg.feed_forward.size() == NUM_JOINTS);

            std::array<bool, NUM_JOINTS> filled{};
            for (size_t i = 0; i < NUM_JOINTS; ++i) {
                auto it = joint_name_to_idx_.find(msg.joint_names[i]);
                if (it == joint_name_to_idx_.end()) {
                    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "Unknown joint name in PDFeedForward: " << msg.joint_names[i]);
                    return false;
                }
                size_t j = it->second;
                if (filled[j]) {
                    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "Duplicate joint name in PDFeedForward: " << msg.joint_names[i]);
                    return false;
                }
                filled[j] = true;
                q[j]  = msg.pos_target[i];
                dq[j] = has_vel ? msg.vel_target[i]   : 0.0;
                u[j]  = has_ff  ? msg.feed_forward[i] : 0.0;
                kp[j] = has_kp  ? msg.kp[i]           : kp_[j];
                kd[j] = has_kd  ? msg.kd[i]           : kd_[j];
            }
            for (size_t j = 0; j < NUM_JOINTS; ++j) {
                if (!filled[j]) {
                    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                        "Missing joint in PDFeedForward: " << ThemisJointNames()[j]);
                    return false;
                }
            }
            return true;
        }

        /// Cosine-ramp the joint targets from ramp_start_q_ to user_pose_ over user_pose_transition_duration_.
        void ComputeUserPoseRamp(std::array<double, NUM_JOINTS>& q,
                                 std::array<double, NUM_JOINTS>& kp) {
            const double t = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - user_pose_transition_start_).count();
            const double alpha = std::min(1.0, std::max(0.0, t / user_pose_transition_duration_));
            // Cosine ease-in-out.
            const double s = 0.5 - 0.5 * std::cos(M_PI * alpha);
            for (size_t i = 0; i < NUM_JOINTS; ++i) {
                q[i]  = (1.0 - s) * ramp_start_q_[i] + s * user_pose_[i];
                // Ramp kp from 10 N·m/rad up to its configured value to soften the engagement.
                kp[i] = (1.0 - s) * 10.0 + s * kp_[i];
            }
        }

        void SnapshotCurrentQAsRampStart() {
            std::lock_guard<std::mutex> lk(state_mtx_);
            if (latest_state_.valid) {
                ramp_start_q_ = latest_state_.q;
            } else {
                // No state feedback yet — start from the home pose.
                std::copy(user_pose_.begin(), user_pose_.end(), ramp_start_q_.begin());
            }
        }

        static void RotMat3x3ToQuaternion(const std::array<double, 9>& R,
                                          double& qw, double& qx, double& qy, double& qz) {
            // Shepperd's method (numerically stable across all rotations).
            const double trace = R[0] + R[4] + R[8];
            if (trace > 0.0) {
                const double s = 0.5 / std::sqrt(trace + 1.0);
                qw = 0.25 / s;
                qx = (R[7] - R[5]) * s;
                qy = (R[2] - R[6]) * s;
                qz = (R[3] - R[1]) * s;
            } else if (R[0] > R[4] && R[0] > R[8]) {
                const double s = 2.0 * std::sqrt(1.0 + R[0] - R[4] - R[8]);
                qw = (R[7] - R[5]) / s;
                qx = 0.25 * s;
                qy = (R[1] + R[3]) / s;
                qz = (R[2] + R[6]) / s;
            } else if (R[4] > R[8]) {
                const double s = 2.0 * std::sqrt(1.0 + R[4] - R[0] - R[8]);
                qw = (R[2] - R[6]) / s;
                qx = (R[1] + R[3]) / s;
                qy = 0.25 * s;
                qz = (R[5] + R[7]) / s;
            } else {
                const double s = 2.0 * std::sqrt(1.0 + R[8] - R[0] - R[4]);
                qw = (R[3] - R[1]) / s;
                qx = (R[2] + R[6]) / s;
                qy = (R[5] + R[7]) / s;
                qz = 0.25 * s;
            }
        }

        void ParseFullState(const uint8_t* payload, size_t n) {
            if (n < STATE_NUM_DOUBLES * sizeof(double)) return;

            LatestState s;
            const double* p = reinterpret_cast<const double*>(payload);
            for (size_t i = 0; i < NUM_JOINTS; ++i) s.q[i]  = p[i];
            p += NUM_JOINTS;
            for (size_t i = 0; i < NUM_JOINTS; ++i) s.dq[i] = p[i];
            p += NUM_JOINTS;
            for (size_t i = 0; i < NUM_JOINTS; ++i) s.u[i]  = p[i];
            p += NUM_JOINTS;
            for (size_t i = 0; i < 3; ++i) s.imu_accel[i] = p[i];
            p += 3;
            for (size_t i = 0; i < 3; ++i) s.imu_gyro[i] = p[i];
            p += 3;
            for (size_t i = 0; i < 9; ++i) s.R[i] = p[i];
            p += 9;
            s.timestamp = *p;
            s.valid = true;

            std::lock_guard<std::mutex> lk(state_mtx_);
            latest_state_ = s;
        }

        void VerifyParameters() {
            auto check = [&](const std::vector<double>& v, const char* name) {
                if (v.size() != NUM_JOINTS) {
                    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "[ThemisInterface] " << name << " size " << v.size() << " != " << NUM_JOINTS);
                    throw std::runtime_error(std::string("[ThemisInterface] bad param size: ") + name);
                }
            };
            check(kp_,         "default_kp");
            check(kd_,         "default_kd");
            check(kd_damping_, "default_kd_damping");
            check(user_pose_,  "user_pose");
            if (default_joint_names_.size() != NUM_JOINTS) {
                throw std::runtime_error("[ThemisInterface] default_joint_names size != 28");
            }
        }

        void BuildJointNameIndex() {
            joint_name_to_idx_.clear();
            for (size_t i = 0; i < default_joint_names_.size(); ++i) {
                joint_name_to_idx_.emplace(default_joint_names_[i], i);
            }
        }

        void OpenSocket() {
            CloseSocket();
            sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
            if (sock_fd_ < 0) {
                throw std::runtime_error(std::string("[ThemisInterface] socket() failed: ") + strerror(errno));
            }
            std::memset(&server_addr_, 0, sizeof(server_addr_));
            server_addr_.sin_family = AF_INET;
            server_addr_.sin_port   = htons(udp_port_);
            if (::inet_pton(AF_INET, target_ip_.c_str(), &server_addr_.sin_addr) != 1) {
                throw std::runtime_error(std::string("[ThemisInterface] bad target_ip: ") + target_ip_);
            }
        }

        void CloseSocket() {
            if (sock_fd_ >= 0) {
                ::close(sock_fd_);
                sock_fd_ = -1;
            }
        }

        static void AppendDoubles(std::vector<uint8_t>& buf, const double* src, size_t n) {
            const uint8_t* p = reinterpret_cast<const uint8_t*>(src);
            buf.insert(buf.end(), p, p + n * sizeof(double));
        }

        void SendDatagram(const std::vector<uint8_t>& pkt) { SendDatagram(pkt.data(), pkt.size()); }

        void SendDatagram(const uint8_t* data, size_t n) {
            if (sock_fd_ < 0) return;
            ::sendto(sock_fd_, data, n, 0,
                     reinterpret_cast<const sockaddr*>(&server_addr_), sizeof(server_addr_));
        }

        // ─────────────────────────── state ──────────────────────────────────
        struct LatestState {
            bool   valid = false;
            double timestamp = 0.0;
            std::array<double, NUM_JOINTS> q{}, dq{}, u{};
            std::array<double, 3> imu_accel{};
            std::array<double, 3> imu_gyro{};
            std::array<double, 9> R{{1, 0, 0, 0, 1, 0, 0, 0, 1}};
        };

        // Config
        std::string target_ip_;
        uint16_t    udp_port_                     = 9870;
        double      feedback_rate_hz_             = 100.0;
        double      heartbeat_period_s_           = 2.0;
        double      user_pose_transition_duration_ = 3.0;
        bool        enable_base_orient_cmd_       = false;
        bool        enable_com_pos_cmd_           = false;
        std::vector<double>      kp_, kd_, kd_damping_, user_pose_;
        std::vector<std::string> default_joint_names_;
        std::unordered_map<std::string, size_t> joint_name_to_idx_;

        // Networking
        int                 sock_fd_ = -1;
        sockaddr_in         server_addr_{};
        std::thread         fb_thread_, hb_thread_;
        std::atomic<bool>   running_{false};

        // State cache
        LatestState         latest_state_;
        mutable std::mutex  state_mtx_;

        // FSM
        std::atomic<ThemisFSMState> fsm_state_{ThemisFSMState::INIT};
        std::array<double, NUM_JOINTS> ramp_start_q_{};
        std::chrono::steady_clock::time_point user_pose_transition_start_;

        // Component keys (match the YAML's ros_parameter suffixes minus "_setting")
        const std::string pub_joint_state_key_ = "pub_sensor";
        const std::string pub_imu_state_key_   = "pub_imu";
        const std::string sub_fsm_key_         = "sub_fsm";
        const std::string sub_vel_cmd_key_     = "sub_vel_cmd";
        const std::string timer_sensor_key_    = "timer_sensor";
    };

}  // namespace obelisk
