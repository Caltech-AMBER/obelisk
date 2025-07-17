from math import pi

import numpy as np
from obelisk_control_msgs.msg import PositionSetpoint
from obelisk_estimator_msgs.msg import EstimatedState
from rclpy.lifecycle import LifecycleState, TransitionCallbackReturn

from obelisk_py.core.control import ObeliskController
from obelisk_py.core.obelisk_typing import ObeliskControlMsg, ObeliskEstimatorMsg, is_in_bound

PUB_CONTROL_NAME = "pub_ctrl"

NUM_JOINTS = 6

# Unit: radians
JOINT_LIMITS = np.array([
    [-135, 135],
    [-90, 90],
    [-90, 90],
    [-135, 135],
    [-90, 90],
    [-135, 135]
]) * pi / 180

# Unit: meters
GRIPPER_LIMITS = np.array([0, 0.03])

class D1Controller(ObeliskController):
    """Example position setpoint controller for the Unitree D1 Arm."""

    def __init__(self, node_name: str) -> None:
        """Initialize controller."""
        super().__init__(node_name, PositionSetpoint, EstimatedState)
        self.info = self.get_logger().info # useful for logging
        
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the controller."""
        super().on_configure(state)
        self.start_time = self.get_clock().now()
        return TransitionCallbackReturn.SUCCESS
    
    def update_x_hat(self, x_hat_msg: ObeliskEstimatorMsg) -> None:
        """
        Update the state estimate.
        
        Args:
            x_hat_msg: The Obelisk message containing the 
            state estimate.
        """
        pass

    def compute_control(self) -> ObeliskControlMsg:
        """
        Compute the joint and gripper positions for the 6-DOF+1 robot. 
        
        joint1 to joint6 positions are in radians.
        gripper1 and gripper2 positions are in meters. 
        The gripper1 position is positive. 
        The gripper2 position is the negative of that of gripper1.
        The control message consists of eight inputs for Mujoco to simulate the 
        robot.
        
        Returns:
            obelisk_control_msg: The control message.
        """
        # Computing the control input
        # Grab the current time.
        t = self.t - self.start_time.nanoseconds * 1e-9
        w = 1

        if t < pi / w: 
            u_joints = [0.0 for i in range(NUM_JOINTS)]
            u_grippers = [0.0, 0.0]
        else:
            u_joints = [(0.3 * np.sin(w * t)) for _ in range(NUM_JOINTS)]
            u_gripper1 = 0.015 * np.sin(w * t) + 0.015
            u_grippers = [u_gripper1, -u_gripper1]

        self.limit_joints(u_joints)
        self.limit_grippers(u_grippers)
        u_joints.extend(u_grippers)

        # Create the message
        position_setpoint_msg = PositionSetpoint()
        position_setpoint_msg.u_mujoco = u_joints
        position_setpoint_msg.q_des = u_joints
        self.obk_publishers[PUB_CONTROL_NAME].publish(position_setpoint_msg)
        assert is_in_bound(type(position_setpoint_msg), ObeliskControlMsg)
        return position_setpoint_msg # ignore type checking for now
    
    def limit_joints(self, joints: list) -> None:
        """Modify the joints list such that they are within the joint limits."""
        for i, input in enumerate(joints):
            min_num = JOINT_LIMITS[i][0]
            max_num = JOINT_LIMITS[i][1]
            joints[i] = self.limit(input, min_num, max_num)

    def limit_grippers(self, grippers: list) -> None:
        """Modify the gripper list such that they are within the gripper limits."""
        min_num = GRIPPER_LIMITS[0]
        max_num = GRIPPER_LIMITS[1]
        grippers[0] = self.limit(grippers[0], min_num, max_num)
        grippers[1] = -grippers[0]

    def limit(self, num: float, min_num: float, max_num: float) -> float:
        """Limit `num` to be between `min_num` and `max_num`."""
        return max(min(max_num, num), min_num)
