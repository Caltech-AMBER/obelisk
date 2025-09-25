from typing import Type

import numpy as np
from obelisk_control_msgs.msg import PositionSetpoint
from obelisk_estimator_msgs.msg import EstimatedState
from rclpy.lifecycle import LifecycleState, TransitionCallbackReturn

from obelisk_py.core.control import ObeliskController
from obelisk_py.core.obelisk_typing import ObeliskControlMsg, is_in_bound


class ExamplePositionSetpointController(ObeliskController):
    """Example position setpoint controller."""

    def __init__(self, node_name: str = "example_position_setpoint_controller") -> None:
        """Initialize the example position setpoint controller."""
        super().__init__(node_name, PositionSetpoint, EstimatedState)
        self.declare_parameter("test_param", "default_value")
        self.get_logger().info(
            f"test_param: {self.get_parameter('test_param').get_parameter_value().string_value}"
        )

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the controller. I.e. Declare all variables required to compute the control input."""
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
        """Compute the control signal for the dummy 2-link robot.

        Returns:
            obelisk_control_msg: The control message.
        """
        # computing the control input
        u = np.sin(self.t)  # example state-independent control input

        # setting the message
        position_setpoint_msg = PositionSetpoint()
        position_setpoint_msg.u_mujoco = [u]
        position_setpoint_msg.q_des = [u]
        self.obk_publishers["pub_ctrl"].publish(position_setpoint_msg)
        assert is_in_bound(type(position_setpoint_msg), ObeliskControlMsg)
        return position_setpoint_msg  # type: ignore
