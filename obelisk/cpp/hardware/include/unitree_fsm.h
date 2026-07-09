#pragma once
#include <cstdint>
#include <rclcpp/time.hpp>


enum class ExecFSMState : uint8_t {
    INIT = 0,
    UNITREE_HOME = 1,
    USER_POSE = 2,
    USER_CTRL = 3,
    UNITREE_VEL_CTRL = 4,
    DAMPING = 5,
    ESTOP = 6
};


const std::unordered_map<ExecFSMState, std::string> TRANSITION_STRINGS = {
    {ExecFSMState::INIT, "INIT"},
    {ExecFSMState::UNITREE_HOME, "UNITREE_HOME"},
    {ExecFSMState::USER_POSE, "USER_POSE"},
    {ExecFSMState::USER_CTRL, "LOW_LEVEL_CONTROL"},
    {ExecFSMState::UNITREE_VEL_CTRL, "HIGH_LEVEL_CONTROL"},
    {ExecFSMState::DAMPING, "DAMPING"},
    {ExecFSMState::ESTOP, "ESTOP"}
};


// Leading-edge log rate-limiter for the FSM command/transition prints. Damping and
// ESTOP are (intentionally) NOT command-debounced, so the joystick republishes them
// every callback tick while the button is held -- which floods the console. This
// debounces the LOG LINE ONLY (never the publish / transition / throw): it emits when
// the key changes (a fresh command shows up immediately) OR when the period elapses
// (a slow reminder if a button is held), and suppresses same-key repeats in between.
// Period reuses the joystick's existing 0.5 s FSM command gate so the log never
// outpaces the commands.
constexpr double FSM_LOG_DEBOUNCE_SEC = 0.5;

struct FsmLogDebounce {
    rclcpp::Time last;
    int key = -1;
    bool inited = false;

    // Returns true if the caller should emit its log line now.
    bool ShouldLog(const rclcpp::Time& now, int key_, double period_sec) {
        if (!inited || key_ != key || (now - last).seconds() > period_sec) {
            last = now;
            key = key_;
            inited = true;
            return true;
        }
        return false;
    }
};