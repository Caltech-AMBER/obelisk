from typing import List, Optional

import rclpy
from obelisk_estimator_msgs.msg import EstimatedState
from obelisk_sensor_msgs.msg import JointEncoders
from rclpy.executors import SingleThreadedExecutor
from rclpy.lifecycle import LifecycleState, TransitionCallbackReturn

from obelisk_py.estimation import ObeliskEstimator


class JointEncodersPassthroughEstimator(ObeliskEstimator):
    """Passthrough estimator for joint encoder sensors."""

    def __init__(self) -> None:
        """Initialize the joint encoders passthrough estimator."""
        super().__init__("joint_encoders_passthrough_estimator")

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the estimator."""
        super().on_configure(state)
        self.joint_encoder_values = None
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate the estimator."""
        super().on_activate(state)
        self.joint_encoder_values = None
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Clean up the estimator."""
        super().on_cleanup(state)
        del self.joint_encoder_values
        return TransitionCallbackReturn.SUCCESS

    def joint_encoder_callback(self, msg: JointEncoders) -> None:
        """Callback for joint encoder messages."""
        self.joint_encoder_values = msg.values

    def compute_state_estimate(self) -> EstimatedState:
        """Compute the state estimate."""
        estimate_state_msg = EstimatedState()
        estimate_state_msg.x_hat = self.joint_encoder_values
        self.publisher_est.publish(estimate_state_msg)
        return estimate_state_msg


def main(args: Optional[List] = None) -> None:
    """Main entrypoint."""
    rclpy.init(args=args)
    passthrough_estimator = JointEncodersPassthroughEstimator()
    executor = SingleThreadedExecutor()
    executor.add_node(passthrough_estimator)
    executor.spin()
    passthrough_estimator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
