import numpy as np
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

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the controller."""
        super().on_figure(state)
        self.x_hat = None # FIXME: should say self.joint_pos = None
        return TransitionCallbackReturn.SUCCESS
    
    def update_x_hat(self, x_hat_msg: ObeliskEstimatorMsg) -> None:
        """
        Update the state estimate.
        
        Args:
            x_hat_msg (ObeliskEstimatorMsg): The Obelisk message containing the state estimate.
        """
        pass

    def compute_control(self) -> ObeliskControlMsg:
        """
        Compute the control signal for the 6-DOF+1 robot.
        
        Returns:
            obelisk_control_msg (ObeliskControlMsg): The control message.
        """
        # Computing the control input
        u = [(0.3 * np.sin(self.t)) for _ in range(8)] # example state-independent control input
        # Setting the message
        position_setpoint_msg = PositionSetpoint()
        position_setpoint_msg.u_mujoco = u
        position_setpoint_msg.q_des = u
        self.obk_publishers["pub_ctrl"].publish(position_setpoint_msg)
        assert is_in_bound(type(position_setpoint_msg), ObeliskControlMsg)
        return position_setpoint_msg # type: ignore. FIXME: ignore what?
