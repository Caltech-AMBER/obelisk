from typing import Union

from obelisk_estimator_msgs.msg import EstimatedState
from obelisk_sensor_msgs.msg import JointEncoders
from rclpy.lifecycle import LifecycleState, TransitionCallbackReturn

from obelisk_py.core.estimation import ObeliskEstimator


class JointEncodersPassthroughEstimator(ObeliskEstimator):
    """Passthrough estimator for joint encoder sensors."""

    def __init__(self, node_name: str = "joint_encoders_passthrough_estimator") -> None:
        """Initialize the joint encoders passthrough estimator."""
        super().__init__(node_name)
        self.register_obk_subscription(
            "sub_sensor_setting",
            self.joint_encoder_callback,  # type: ignore
            key="subscriber_sensor",  # key can be specified here or in the config file
            msg_type=JointEncoders,
        )

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the estimator."""
        super().on_configure(state)
        self.joint_encoder_values = None
        return TransitionCallbackReturn.SUCCESS

    def joint_encoder_callback(self, msg: JointEncoders) -> None:
        """Callback for joint encoder messages."""
        self.joint_encoder_values = msg.y

    def compute_state_estimate(self) -> Union[EstimatedState, None]:
        """Compute the state estimate."""
        estimated_state_msg = EstimatedState()
        if self.joint_encoder_values is not None:
            estimated_state_msg.x_hat = self.joint_encoder_values
            self.obk_publishers["publisher_ctrl"].publish(estimated_state_msg)
            return estimated_state_msg
