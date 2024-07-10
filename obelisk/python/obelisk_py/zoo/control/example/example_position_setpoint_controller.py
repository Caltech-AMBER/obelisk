import numpy as np
from obelisk_control_msgs.msg import PositionSetpoint
from rclpy.lifecycle import LifecycleState, TransitionCallbackReturn

from obelisk_py.core.control import ObeliskController
from obelisk_py.core.obelisk_typing import ObeliskControlMsg, ObeliskEstimatorMsg, is_in_bound


class ExamplePositionSetpointController(ObeliskController):
    """Example position setpoint controller."""

    def __init__(self, node_name: str = "example_position_setpoint_controller") -> None:
        """Initialize the example position setpoint controller."""
        super().__init__(node_name)

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the controller."""
        super().on_configure(state)
        self.x_hat = None
        return TransitionCallbackReturn.SUCCESS

    def update_x_hat(self, x_hat_msg: ObeliskEstimatorMsg) -> None:
        """Update the state estimate.

        Parameters:
            x_hat_msg: The Obelisk message containing the state estimate.
        """
        pass  # do nothing

    def compute_control(self) -> ObeliskControlMsg:
        """Compute the control signal for the dummy 2-link robot.

        Returns:
            obelisk_control_msg: The control message.
        """
        # computing the control input
        u = 10 * np.sin(10 * self.t)  # example state-independent control input

        # setting the message
        position_setpoint_msg = PositionSetpoint()
        position_setpoint_msg.u = [u]    # TODO
        self.obk_publishers["pub_ctrl"].publish(position_setpoint_msg)
        assert is_in_bound(type(position_setpoint_msg), ObeliskControlMsg)
        return position_setpoint_msg  # type: ignore
