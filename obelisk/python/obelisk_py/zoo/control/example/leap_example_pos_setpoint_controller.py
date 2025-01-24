from typing import Type

import numpy as np
from obelisk_control_msgs.msg import PositionSetpoint
from obelisk_estimator_msgs.msg import EstimatedState
from rclpy.lifecycle import LifecycleState, TransitionCallbackReturn

from obelisk_py.core.control import ObeliskController
from obelisk_py.core.obelisk_typing import ObeliskControlMsg, is_in_bound


class LeapExamplePositionSetpointController(ObeliskController):
    """Example position setpoint controller."""

    def __init__(self, node_name: str = "leap_example_position_setpoint_controller") -> None:
        """Initialize the example position setpoint controller."""
        super().__init__(node_name, PositionSetpoint, EstimatedState)

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the controller."""
        super().on_configure(state)
        self.joint_pos = None
        return TransitionCallbackReturn.SUCCESS

    def update_x_hat(self, x_hat_msg: Type) -> None:
        """Update the state estimate.

        Parameters:
            x_hat_msg: The Obelisk message containing the state estimate.
        """
        pass  # do nothing

    def compute_control(self) -> Type:
        """Compute the control signal for the LEAP hand.

        Returns:
            The control message.
        """
        # setting the message
        position_setpoint_msg = PositionSetpoint()
        position_setpoint_msg.u_mujoco = [(0.3 * np.sin(self.t)) for _ in range(16)]  # example state-independent input
        position_setpoint_msg.q_des = [(0.3 * np.sin(self.t)) for _ in range(16)]  # example state-independent input
        self.obk_publishers["pub_ctrl"].publish(position_setpoint_msg)
        assert is_in_bound(type(position_setpoint_msg), ObeliskControlMsg)
        return position_setpoint_msg  # type: ignore
