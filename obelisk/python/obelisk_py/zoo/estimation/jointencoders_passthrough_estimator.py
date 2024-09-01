from typing import Union

from obelisk_estimator_msgs.msg import EstimatedState
from obelisk_sensor_msgs.msg import ObkJointEncoders
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
            key="sub_sensor",  # key can be specified here or in the config file
            msg_type=ObkJointEncoders,
        )
        self._last_msg = None

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the estimator."""
        super().on_configure(state)
        self.joint_encoder_values = None
        self.joint_encoder_values_prev = None
        self.t_prev = None
        return TransitionCallbackReturn.SUCCESS

    def joint_encoder_callback(self, msg: ObkJointEncoders) -> None:
        """Callback for joint encoder messages."""
        self.joint_encoder_values = msg.joint_pos

    def compute_state_estimate(self) -> Union[EstimatedState, None]:
        """Compute the state estimate."""
        estimated_state_msg = EstimatedState()
        estimated_state_msg.header.stamp = self.get_clock().now().to_msg()

        if self.joint_encoder_values is None or (self.joint_encoder_values == self.joint_encoder_values_prev):
            if self._last_msg is not None:
                return self._last_msg
            else:
                return None
        else:
            estimated_state_msg.q_joints = self.joint_encoder_values
            if self.joint_encoder_values_prev is not None:
                t = self.t
                dt = t - self.t_prev
                estimated_state_msg.v_joints = [
                    (q - q_prev) / dt for q, q_prev in zip(self.joint_encoder_values, self.joint_encoder_values_prev)
                ]
                self.joint_encoder_values_prev = self.joint_encoder_values
                self.t_prev = t
            else:
                estimated_state_msg.v_joints = [0.0] * len(self.joint_encoder_values)
                self.joint_encoder_values_prev = self.joint_encoder_values
                self.t_prev = self.t
            estimated_state_msg.joint_names = [
                "if_mcp", "if_rot", "if_pip", "if_dip", "mf_mcp", "mf_rot", "mf_pip", "mf_dip",
                "rf_mcp", "rf_rot", "rf_pip", "rf_dip", "th_cmc", "th_axl", "th_mcp", "th_ipl",
            ]  # fmt: skip
            self.obk_publishers["pub_est"].publish(estimated_state_msg)
            self._last_msg = estimated_state_msg
            return estimated_state_msg
