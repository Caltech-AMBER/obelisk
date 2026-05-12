#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace obelisk {

    /**
     * @brief Execution FSM states for the Westwood Themis interface.
     *
     * Values are sent as the single-byte payload of MSG_FSM_TRANSITION on the
     * UDP wire (see themis_interface.h). The matching enum is in
     * obelisk_westwood_cpp/server/themis_obk_server.py (class FSMState).
     */
    enum class ThemisFSMState : uint8_t {
        INIT           = 0,  ///< Sockets up, threads running. Server has motors disabled.
        USER_POSE      = 1,  ///< Client-side cosine ramp to user_pose_. Server passes through.
        USER_CTRL      = 2,  ///< Client forwards PDFeedForward → FULL_BODY_CMD (user-ctrl mode).
        HIGH_LEVEL_VEL = 3,  ///< Client forwards VelocityCommand → BASE_VEL_CMD (high-level mode).
        DAMPING        = 4,  ///< Server runs wbc_api.damping_motors per chain. No setpoints from us.
        ESTOP          = 5,  ///< Sticky DAMPING. Server refuses further transitions.
    };

    /// Pretty-print map. Mirrors the style of unitree_fsm.h's TRANSITION_STRINGS.
    inline const std::unordered_map<ThemisFSMState, std::string>& ThemisFSMNames() {
        static const std::unordered_map<ThemisFSMState, std::string> NAMES = {
            {ThemisFSMState::INIT,           "INIT"},
            {ThemisFSMState::USER_POSE,      "USER_POSE"},
            {ThemisFSMState::USER_CTRL,      "USER_CTRL"},
            {ThemisFSMState::HIGH_LEVEL_VEL, "HIGH_LEVEL_VEL"},
            {ThemisFSMState::DAMPING,        "DAMPING"},
            {ThemisFSMState::ESTOP,          "ESTOP"},
        };
        return NAMES;
    }

    /**
     * @brief Legal FSM transitions.
     *
     * Cross-mode rules (enforced server-side per the dev manual §3.6.1):
     *   - HIGH_LEVEL_VEL → USER_POSE: server zeroes walking velocity and switches
     *     locomotion mode to standing before honoring the transition.
     *   - USER_POSE → HIGH_LEVEL_VEL: server toggles locomotion mode to walking.
     *   - You cannot jump directly between USER_CTRL and HIGH_LEVEL_VEL — joint
     *     authority and locomotion authority are mutually exclusive on the robot.
     */
    inline const std::unordered_map<ThemisFSMState, std::vector<ThemisFSMState>>& ThemisFSMTransitions() {
        static const std::unordered_map<ThemisFSMState, std::vector<ThemisFSMState>> T = {
            {ThemisFSMState::INIT,
                {ThemisFSMState::USER_POSE, ThemisFSMState::DAMPING, ThemisFSMState::ESTOP}},
            {ThemisFSMState::USER_POSE,
                {ThemisFSMState::USER_CTRL, ThemisFSMState::HIGH_LEVEL_VEL,
                 ThemisFSMState::DAMPING, ThemisFSMState::ESTOP}},
            {ThemisFSMState::USER_CTRL,
                {ThemisFSMState::USER_POSE, ThemisFSMState::DAMPING, ThemisFSMState::ESTOP}},
            {ThemisFSMState::HIGH_LEVEL_VEL,
                {ThemisFSMState::USER_POSE, ThemisFSMState::DAMPING, ThemisFSMState::ESTOP}},
            {ThemisFSMState::DAMPING,
                {ThemisFSMState::USER_POSE, ThemisFSMState::ESTOP}},
            {ThemisFSMState::ESTOP, {}},
        };
        return T;
    }

}  // namespace obelisk
