import numpy as np
from franky import Affine, CartesianMotion, JointMotion, ReferenceType
from obelisk_control_msgs.msg import PoseSetpoint, PositionSetpoint
from obelisk_sensor_msgs.msg import ObkFramePose, ObkJointEncoders

from obelisk_py.core.robot import ObeliskRobot
from obelisk_py.zoo.hardware.robots.fr3.fr3_utils import setup_robot


class ObeliskFR3Robot(ObeliskRobot):
    """The FR3 hardware driver in Obelisk."""

    def __init__(self, node_name: str) -> None:
        """Initialize the Obelisk FR3 robot."""
        super().__init__(node_name, PositionSetpoint)
        self._ready = False
        self.register_obk_publisher(
            "pub_joints_setting",
            ObkJointEncoders,
            key="pub_joints",
        )
        self.register_obk_publisher(
            "pub_ee_pose_setting",
            ObkFramePose,
            key="pub_ee_pose",
        )
        self.register_obk_timer(
            "timer_sensor_setting",
            self.get_state,
            key="timer_sensor",
        )

        # initialize
        self.declare_parameter("username")
        self.declare_parameter("password")
        self.declare_parameter("robot_ip", "172.16.0.2")
        self.declare_parameter("ctrl_mode", "ee")

        username = self.get_parameter("username").get_parameter_value().string_value
        password = self.get_parameter("password").get_parameter_value().string_value
        robot_ip = self.get_parameter("robot_ip").get_parameter_value().string_value
        self.robot, self.gripper, self.web_session = setup_robot(robot_ip, username, password)
        self.ctrl_mode = self.get_parameter("ctrl_mode").get_parameter_value().string_value
        assert self.ctrl_mode in ["ee", "joint"], "Invalid control mode specified."
        self._ready = True

    def apply_control(self, control_msg: PositionSetpoint | PoseSetpoint) -> None:
        """Write the control message to the FR3."""
        if not self._ready:
            return None

        self.robot.recover_from_errors()
        if self.ctrl_mode == "ee":
            assert isinstance(control_msg, PoseSetpoint)
            motion = CartesianMotion(
                Affine(
                    np.array([control_msg[0], control_msg[1], control_msg[2]]),
                    np.array([control_msg[4], control_msg[5], control_msg[6], control_msg[3]]),  # (w, x, y, z) in
                ),
                ReferenceType.Absolute,  # TODO: expose an option for relative motion
            )
        elif self.ctrl_mode == "joint":
            assert isinstance(control_msg, PositionSetpoint)
            motion = JointMotion(np.array(control_msg.q_des))
        else:
            raise ValueError(f"Invalid control mode: {self.ctrl_mode}. Valid modes are 'ee' and 'joint'.")
        self.robot.move(motion, asynchronous=True)

    def get_state(self) -> None:
        """Read the joint encoders and publish a sensor message."""
        if not self._ready:
            return None

        # joints
        q = self.robot.current_joint_state.position.tolist()
        dq = self.robot.current_joint_state.velocity.tolist()

        joint_encoders = ObkJointEncoders()
        joint_encoders.joint_pos = q
        joint_encoders.joint_vel = dq
        joint_encoders.joint_names = ["q0", "q1", "q2", "q3", "q4", "q5", "q6"]

        # ee
        ee_pose = self.robot.current_cartesian_state.pose.end_effector_pose
        ee_translation = ee_pose.translation
        ee_quaternion = ee_pose.quaternion
        ee_vel = self.robot.current_cartesian_state.velocity.end_effector_twist
        ee_linvel = ee_vel.linear
        ee_angvel = ee_vel.angular

        frame_pose = ObkFramePose()
        frame_pose.position.x = ee_translation[0]
        frame_pose.position.y = ee_translation[1]
        frame_pose.position.z = ee_translation[2]
        frame_pose.orientation.x = ee_quaternion[0]  # the franky API returns quaternion in [x, y, z, w] order
        frame_pose.orientation.y = ee_quaternion[1]
        frame_pose.orientation.z = ee_quaternion[2]
        frame_pose.orientation.w = ee_quaternion[3]
        frame_pose.linear_velocity.x = ee_linvel[0]
        frame_pose.linear_velocity.y = ee_linvel[1]
        frame_pose.linear_velocity.z = ee_linvel[2]
        frame_pose.angular_velocity.x = ee_angvel[0]
        frame_pose.angular_velocity.y = ee_angvel[1]
        frame_pose.angular_velocity.z = ee_angvel[2]
        frame_pose.frame_name = "ee_tcp"

        # publish
        self.obk_publishers["pub_joints"].publish(joint_encoders)
        self.obk_publishers["pub_ee_pose"].publish(frame_pose)


if __name__ == "__main__":
    robot_ip = "172.16.0.2"
    username = "persephone"
    password = "persephone"
    robot, gripper = setup_robot(robot_ip, username, password)
    breakpoint()
