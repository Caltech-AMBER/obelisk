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