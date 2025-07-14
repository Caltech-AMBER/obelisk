from typing import Any

from obelisk_estimator_msgs.msg import EstimatedState
from obelisk_sensor_msgs.msg import ObkJointEncoders

from obelisk_py.core.estimation import ObeliskEstimator

SUB_SENSOR_NAME = "sub_sensor_setting"
SUB_SENSOR_KEY = "subsensor"
PUB_ESTIMATOR_NAME = "pub_est"
BASE_LINK_NAME = "base_link"

class D1Estimator(ObeliskEstimator):
    """Passthrough estimator for joint encoder sensors."""
    def __init__(self, node_name: str="d1_estimator") -> None:
        """Initialize the passthrough estimator."""
        super().__init__(node_name, EstimatedState)
        self.register_obk_subscription(
            SUB_SENSOR_NAME,
            self.joint_encoder_callback, # type: ignore
            ObkJointEncoders,
            key=SUB_SENSOR_KEY, # key can be specified here or in the config file
        )

        self.received_joint_encoders = False

    def joint_encoder_callback(self, msg: ObkJointEncoders) -> None:
        """Configure the estimator."""
        self.joint_pos = msg.joint_pos
        self.joint_vel = msg.joint_vel
        self.joint_names = msg.joint_names
        self.received_joint_encoders = True

    def compute_state_estimate(self) -> Any:
        """Compute the state estimate."""
        if not self.received_joint_encoders:
            return
        
        msg = EstimatedState()
        msg.q_joints = self.joint_pos
        msg.v_joints = self.joint_vel
        msg.joint_names = self.joint_names
        msg.base_link_name = BASE_LINK_NAME
        msg.header.stamp = self.get_clock().now().to_msg()
        self.obk_publishers[PUB_ESTIMATOR_NAME].publish(msg)
        return msg




