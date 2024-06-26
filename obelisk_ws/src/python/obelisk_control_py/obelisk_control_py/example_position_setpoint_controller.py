from typing import List, Optional

import numpy as np
import rclpy
from obelisk_control_msgs.msg import PositionSetpoint
from rclpy.executors import SingleThreadedExecutor

from obelisk_py.control import ObeliskController
from obelisk_py.obelisk_typing import ObeliskControlMsg, ObeliskEstimatorMsg


class ExamplePositionSetpointController(ObeliskController):
    """Example position setpoint controller."""

    def __init__(self) -> None:
        """Initialize the example position setpoint controller."""
        super().__init__("example_position_setpoint_controller")

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
        t = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9
        u = 0.1 * np.sin(t)  # example state-independent control input

        # setting the message
        position_setpoint_msg = PositionSetpoint()
        position_setpoint_msg.u = u
        self.publisher_ctrl.publish(position_setpoint_msg)
        return position_setpoint_msg


def main(args: Optional[List] = None) -> None:
    """Main entrypoint."""
    rclpy.init(args=args)
    example_position_setpoint_controller = ExamplePositionSetpointController()
    executor = SingleThreadedExecutor()
    executor.add_node(example_position_setpoint_controller)
    executor.spin()
    example_position_setpoint_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
