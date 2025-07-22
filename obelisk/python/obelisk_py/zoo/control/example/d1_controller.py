from typing import Union, Optional
from math import pi

import numpy as np
from obelisk_control_msgs.msg import PositionSetpoint
from obelisk_estimator_msgs.msg import EstimatedState
from rclpy.lifecycle import LifecycleState, TransitionCallbackReturn

from obelisk_py.core.control import ObeliskController
from obelisk_py.core.obelisk_typing import ObeliskControlMsg, ObeliskEstimatorMsg, is_in_bound

### CONSTANTS ###
"""Publishers"""
PUB_CONTROL_NAME = "pub_ctrl"

"""Timer parameters"""
TIMER_CTRL_NAME = "timer_ctrl_setting"
TIMER_PERIOD_SEC_KEY = "timer_period_sec"

"""Robot Info"""
NUM_JOINTS = 6
NUM_CONTROL_INPUTS = 8
QG_INIT = np.zeros(NUM_JOINTS)
GRIPPERG_INIT = 0 # goal gripper position (meters)

"""Control Inputs"""
W = 0.25
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

### HELPER FUNCTIONS ###
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

class D1Controller(ObeliskController):
    """Example position setpoint controller for the Unitree D1 Arm."""

    def __init__(self, node_name: str) -> None:
        """Initialize controller."""
        super().__init__(node_name, PositionSetpoint, EstimatedState)
        self.info = self.get_logger().info # useful for logging
   
        self.q0 = None # initialize the starting joint positions
        self.gripper0 = None # initialize the starting gripper position

        self._qd = None # initialize the desired joint positions
        self._gripperd = None # initialize the desired gripper position
        
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the controller."""
        super().on_configure(state)
        self.start_time = self.get_clock().now()
        self.dt = self.get_timer_period_sec(TIMER_CTRL_NAME)
        return TransitionCallbackReturn.SUCCESS
    
    def get_timer_period_sec(self, name: str, key: str=TIMER_PERIOD_SEC_KEY) -> float:
        """
        Returns the `timer_period_sec` associated with parameter `name`.

        The `timer_period_sec` is the duration in seconds between consecutive
        invocations of the timer with the parameter `name`. 
        """
        string = self.get_parameter(name).get_parameter_value().string_value
        pairs = dict(pair.split(':') for pair in string.split(','))
        dt = float(pairs[TIMER_PERIOD_SEC_KEY])
        return dt
    
    def update_x_hat(self, x_hat_msg: ObeliskEstimatorMsg) -> None:
        """
        Update the state estimate.
        
        Args:
            x_hat_msg: The Obelisk message containing the 
            state estimate.
        """
        if len(x_hat_msg.q_joints) != NUM_CONTROL_INPUTS: # this may occur when simulating the robot
            return
        
        # Update the state
        servo_state = np.array(x_hat_msg.q_joints)
        self._q = servo_state[:NUM_JOINTS]
        self._gripper = servo_state[-2]  # gripper position is positive

        if self.q0 is None:
            self.init_kinematic_parameters(self._q, self._gripper)

    def init_kinematic_parameters(self, q0: np.ndarray, gripper0: float) -> None:
        """
        Initialize parameters for computing the kinematics of
        the arm.
        
        Initialize position of gripper.

        Args:
            q0 ((6,)-shape np.ndarray): initial joint positions
            gripper0 (float): initial gripper position
        """  # noqa: D205
        self.q0 = q0 # the initial joint positions
        self.gripper0 = gripper0 # the initial gripper position
        
        self.qd = q0 # the desired joint positions
        self.gripperd = gripper0 # the desired gripper position

        self.info("Control inputs: %s" % self.control_inputs)
    
    @property
    def qd(self) -> Optional[np.ndarray]:
        """The desired joint positions (radians)."""
        return self._qd
    
    @qd.setter
    def qd(self, value: np.ndarray) -> None:
        if value is not None:
            limit_joints(value)
        self._qd = value

    @property
    def gripperd(self) -> float:
        """
        The desired gripper position (meters) i.e. the distance between 
        the center of the clamps and the inner face of a clamp.
        """  # noqa: D205
        return self._gripperd

    @gripperd.setter
    def gripperd(self, value: float) -> None:
        if value is not None:
            value = limit_gripper(value)
        self._gripperd = value

    @property
    def control_inputs(self) -> np.ndarray:
        """
        The control inputs for the Mujoco simulator and computing inverse
        kinematics.
        """  # noqa: D205
        return np.hstack((self._qd, self._gripperd, -self._gripperd))

    @control_inputs.setter
    def control_inputs(self, value: np.ndarray) -> None:
        """
        Sets the private variables for computing the control input.

        Args:
            value ((8,) np.ndarray): first six values are the joint positions.
            Last two values are the gripper positions.
        """
        self.qd = value[:NUM_JOINTS]
        self.gripperd = value[-2]

    def compute_control(self) -> Optional[ObeliskControlMsg]:
        """
        Compute the joint and gripper positions for the 6-DOF+1 robot. 
        
        joint1 to joint6 positions are in radians.
        gripper1 and gripper2 positions are in meters. 
        The gripper1 position is positive. 
        The gripper2 position is the negative of that of gripper1.
        The control message consists of eight inputs for Mujoco to simulate the 
        robot.
        
        Returns:
            obelisk_control_msg: The control message.
        """
        if self.q0 is None:
            return
        
        # Computing the control input
        # Grab the current time.
        t = self.t - self.start_time.nanoseconds * 1e-9
    
        # FIXME: 
        if t < INIT_TIME:
            # Compute the desired servo positions
            (self.qd, _) = goto(t, INIT_TIME, self.q0, QG_INIT) # set desired joint positions
            (self.gripperd, _) = goto(t, INIT_TIME, self.gripper0, GRIPPERG_INIT) # set desired gripper position
        else:
            self.qd = np.array([(0.3 * np.sin(W * t)) for _ in range(NUM_JOINTS)])
            self.gripperd = 0.015 * np.sin(W * t) + 0.015

        control_inputs = self.control_inputs.tolist()
        # Create the message
        position_setpoint_msg = PositionSetpoint()
        position_setpoint_msg.u_mujoco = control_inputs
        position_setpoint_msg.q_des = control_inputs
        self.info("t: %f, control_inputs: %s" % (t, control_inputs))
        self.obk_publishers[PUB_CONTROL_NAME].publish(position_setpoint_msg)
        assert is_in_bound(type(position_setpoint_msg), ObeliskControlMsg)
        return position_setpoint_msg # ignore type checking for now
    
   