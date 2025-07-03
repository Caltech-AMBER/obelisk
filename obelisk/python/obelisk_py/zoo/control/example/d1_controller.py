import numpy as np
from math import pi

from obelisk_control_msgs.msg import PositionSetpoint
from obelisk_estimator_msgs.msg import EstimatedState
from rclpy.lifecycle import LifecycleState, TransitionCallbackReturn

from obelisk_py.core.control import ObeliskController
from obelisk_py.core.obelisk_typing import ObeliskControlMsg, ObeliskEstimatorMsg, is_in_bound

class D1Controller(ObeliskController):
    """Example position setpoint controller for the Unitree D1 Arm."""

    def __init__(self, node_name: str="d1_controller") -> None:
        """Initialize controller."""
        super().__init__(node_name, PositionSetpoint, EstimatedState)
        self.start_time = self.get_clock().now()

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the controller."""
        super().on_configure(state)
        return TransitionCallbackReturn.SUCCESS
    
    def update_x_hat(self, x_hat_msg: ObeliskEstimatorMsg) -> None:
        """
        Update the state estimate.
        
        Args:
            x_hat_msg (ObeliskEstimatorMsg): The Obelisk message containing the 
            state estimate.
        """
        pass

    def compute_control(self) -> ObeliskControlMsg:
        """
        Compute the joint positions for the 6-DOF+1 robot. 
        
        joint0 to joint5 positions are in radians.
        joint6 and joint6_2 are in meters. 
        They control the gripper.
        The position of motor 6 is positive. 
        The position of motor 6_2 is the negative of that of motor 6.
        The control message contains eight joints for Mujoco to simulate the 
        robot.
        
        Returns:
            obelisk_control_msg (ObeliskControlMsg): The control message.
        """
        # Computing the control input
        # Grab the current time.
        # `t` doesn't start at 0. It starts at 0.13 seconds.
        t = self.t - self.start_time.nanoseconds * 1e-9 
        w = 1

        if t < pi / w: 
            u = np.zeros(8).astype(float).tolist() 
        else:
            u = [(0.3 * np.sin(w * t)) for _ in range(6)]
            u_gripper = 0.015 * np.sin(w * t) + 0.015
            u.append(u_gripper)
            u.append(-u_gripper)

        # Create the message
        position_setpoint_msg = PositionSetpoint()
        position_setpoint_msg.u_mujoco = u
        position_setpoint_msg.q_des = u
        self.obk_publishers["pub_ctrl"].publish(position_setpoint_msg)
        assert is_in_bound(type(position_setpoint_msg), ObeliskControlMsg)
        return position_setpoint_msg # ignore type checking for now
