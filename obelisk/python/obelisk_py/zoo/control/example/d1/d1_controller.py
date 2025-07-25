from math import pi
from typing import Optional, Union

import numpy as np
from obelisk_control_msgs.msg import PositionSetpoint
from obelisk_estimator_msgs.msg import EstimatedState
from rclpy.lifecycle import LifecycleState, TransitionCallbackReturn

from obelisk_py.core.control import ObeliskController
from obelisk_py.core.obelisk_typing import ObeliskControlMsg, ObeliskEstimatorMsg, is_in_bound
from obelisk_py.zoo.control.example.d1.constants import *
from obelisk_py.zoo.control.example.d1.utils.ControlUtils import limit_gripper, limit_joints, goto
from obelisk_py.zoo.control.example.d1.utils.RecordingUtils import initialize_folder, record_data


class D1Controller(ObeliskController):
    """Example position setpoint controller for the Unitree D1 Arm."""

    def __init__(self, node_name: str) -> None:
        """Initialize controller."""
        super().__init__(node_name, PositionSetpoint, EstimatedState)
        self.info = self.get_logger().info # useful for logging

        self.start_time  = None
   
        self.q0 = None # initialize the starting joint positions
        self.gripper0 = None # initialize the starting gripper position

        self._qd = None # initialize the desired joint positions
        self._gripperd = None # initialize the desired gripper position

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the controller."""
        super().on_configure(state)
        # Declare ros2 parameter
        self.declare_parameter(RECORDING_STR, False)
        self.recording = self.get_parameter(RECORDING_STR).get_parameter_value().bool_value
        self.info("Recording data: %s" % self.recording)
        # Initialize folder for storing data
        if self.recording:
            initialize_folder()

        self.dt = self.get_timer_period_sec(TIMER_CTRL_NAME)
        return TransitionCallbackReturn.SUCCESS
    
    def update_x_hat(self, x_hat_msg: ObeliskEstimatorMsg) -> None:
        """
        Update the state estimate.
        
        Args:
            x_hat_msg: The Obelisk message containing the 
            state estimate.
        """
        # Update the state
        state = np.array(x_hat_msg.q_joints) # (8,)-shape np.ndarray
        servo_state = state[:-1]
        self._q = servo_state[:NUM_JOINTS]
        self._gripper = servo_state[-1]  # gripper position is positive

        if self.q0 is None:
            self.init_kinematic_parameters(self._q, self._gripper)

        # Record data
        if self.recording:
            t = x_hat_msg.header.stamp.sec - self.start_time
            record_data(filepath=SERVO_STATE_FILE_PATH, t=t, servo_data=servo_state.tolist())

    def compute_control(self) -> Optional[ObeliskControlMsg]: # type: ignore
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
        if self.start_time is None:
            return
        
        t = self.t - self.start_time
        if t < INIT_TIME:
            # Compute the desired servo positions
            (self.qd, _) = goto(t, INIT_TIME, self.q0, QG_INIT) # set desired joint positions
            (self.gripperd, _) = goto(t, INIT_TIME, self.gripper0, GRIPPERG_INIT) # set desired gripper position
        else:
            self.qd = np.array([(0.3 * np.sin(W * t)) for _ in range(NUM_JOINTS)])
            self.gripperd = 0.015 * np.sin(W * t) + 0.015

        control_inputs = self.control_inputs.tolist()
        # self.info("t: %f, control inputs: %s" % (t, control_inputs))
        
        # Create the message
        position_setpoint_msg = PositionSetpoint()
        position_setpoint_msg.u_mujoco = control_inputs
        position_setpoint_msg.q_des = control_inputs
        self.obk_publishers[PUB_CONTROL_NAME].publish(position_setpoint_msg)
        assert is_in_bound(type(position_setpoint_msg), ObeliskControlMsg)

        # Record the servo command
        if self.recording:
            servo_command = control_inputs[:-1]
            record_data(SERVO_COMMAND_FILE_PATH, t, servo_command)
        return position_setpoint_msg # ignore type checking for now
    
    """HELPER FUNCTIONS BELOW"""
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

        # Time since robot is ready to receive control inputs
        self.start_time = self.get_clock().now().nanoseconds * 1e-9 # .seconds isn't supported in rclpy

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
    
   