from typing import Type

import numpy as np
from obelisk_control_msgs.msg import PositionSetpoint
from obelisk_estimator_msgs.msg import EstimatedState
from rclpy.lifecycle import LifecycleState, TransitionCallbackReturn

from obelisk_py.core.control import ObeliskController
from obelisk_py.core.obelisk_typing import ObeliskControlMsg, is_in_bound


class FR3ExampleJointController(ObeliskController):
    """Example joint controller."""

    def __init__(self, node_name: str = "fr3_example_joint_controller") -> None:
        """Initialize the example fr3 joint controller."""
        super().__init__(node_name, PositionSetpoint, EstimatedState)
        self.reset_pos = [
            0.09162008114028396,
            -0.19826458111314524,
            -0.01990020486871322,
            -2.4732269941140346,
            -0.01307073642274261,
            2.30396583422025,
            0.8480939705504309,
        ]

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
        """Compute the control signal for the FR3.

        Returns:
            The control message.
        """
        position_setpoint_msg = PositionSetpoint()

        # example state-independent input
        position_setpoint_msg.u_mujoco = [self.reset_pos[i] + 0.1 * np.sin(self.t) for i in range(7)]
        position_setpoint_msg.q_des = [self.reset_pos[i] + 0.1 * np.sin(self.t) for i in range(7)]
        self.obk_publishers["pub_ctrl"].publish(position_setpoint_msg)
        assert is_in_bound(type(position_setpoint_msg), ObeliskControlMsg)
        return position_setpoint_msg  # type: ignore
