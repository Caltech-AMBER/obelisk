#pragma once
#include <cstdint>


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