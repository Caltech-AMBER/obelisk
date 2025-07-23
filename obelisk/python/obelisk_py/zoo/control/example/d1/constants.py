import time
from math import pi

import numpy as np

"""Publishers"""
PUB_CONTROL_NAME = "pub_ctrl"

"""Timer parameters"""
TIMER_CTRL_NAME = "timer_ctrl_setting"
TIMER_PERIOD_SEC_KEY = "timer_period_sec"

"""Robot Info"""
NUM_JOINTS = 6
NUM_SERVOS = 7 # 6 servos for the joints; one servo for the gripper
NUM_CONTROL_INPUTS = 8
QG_INIT = np.zeros(NUM_JOINTS)
GRIPPERG_INIT = 0 # goal gripper position (meters)

"""Control Inputs"""
W = 1
INIT_TIME = 2 * pi / W

"""Control Limits"""
# Unit: radians
JOINT_LIMITS = np.array([
    [-135, 135],
    [-90, 90],
    [-90, 90],
    [-135, 135],
    [-90, 90],
    [-135, 135]
]) * pi / 180

# Unit: meters
GRIPPER_LIMITS = np.array([0, 0.03])

"""Recording data"""
RECORDING_STR = "recording"
TIME_STR = time.strftime("%Y%m%d-%H%M%S")
FOLDER_PATH = f"obelisk/python/obelisk_py/zoo/control/example/d1/data/{TIME_STR}"
SERVO_COMMAND_FILE_PATH = f"{FOLDER_PATH}/servo_command.csv"
SERVO_STATE_FILE_PATH = f"{FOLDER_PATH}/servo_state.csv"
HEADER = ['time'] + [f"servo{i + 1}" for i in range(NUM_SERVOS)]