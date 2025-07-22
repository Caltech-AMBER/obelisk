from typing import Optional, Union

import numpy as np

from obelisk_py.zoo.control.example.d1.constants import *

"""Limiting control inputs"""
def limit_joints(joints: Union[list[float], np.ndarray]) -> None:
    """Modify the joints list/array such that it is within the joint limits."""
    for i, input in enumerate(joints):
        min_num = JOINT_LIMITS[i][0]
        max_num = JOINT_LIMITS[i][1]
        joints[i] = limit(input, min_num, max_num)

def limit_gripper(gripper: float) -> float:
    """Return the gripper position such that it is within its limits."""
    min_num = GRIPPER_LIMITS[0]
    max_num = GRIPPER_LIMITS[1]
    gripper = limit(gripper, min_num, max_num)
    return gripper

def limit(num: float, min_num: float, max_num: float) -> float:
    """Limit `num` to be between `min_num` and `max_num`."""
    return max(min(max_num, num), min_num)

"""Trajectory util"""
def goto(
    t: float, 
    T: float, 
    p0: Union[np.ndarray, float], 
    pf: Union[np.ndarray, float]
    ) -> tuple:
    # Compute the current (p,v).
    p = p0 + (pf - p0) * (3 * (t / T) ** 2 - 2 * (t / T) ** 3)
    v = (pf - p0) / T * (6 * (t / T) - 6 * (t / T) ** 2)
    return (p, v)
