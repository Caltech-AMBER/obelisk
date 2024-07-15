from math import cos, sin

import obelisk_estimator_msgs.msg as oem
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from tf2_ros import TransformBroadcaster
from urdf_parser_py.urdf import URDF


class StatePublisher(Node):  # noqa: D101
    # TODO: re-do this node to publish dummy estimated messages so that I can test the whole stack
    def __init__(self) -> None:
        """Basic testing function. Will be removed later."""
        rclpy.init()
        super().__init__("state_publisher")

        qos_profile = QoSProfile(depth=10)
        self.est_state_pub = self.create_publisher(oem.EstimatedState, "estimated_state", qos_profile)
        self.est_state_pub2 = self.create_publisher(oem.EstimatedState, "estimated_state2", qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        self.declare_parameter("robot_description", "")
        self.robot_description = self.get_parameter("robot_description").get_parameter_value().string_value

        self.declare_parameter("robot_description2", "")
        self.robot_description2 = self.get_parameter("robot_description2").get_parameter_value().string_value

        loop_rate = self.create_rate(30)

        angle = 0.0
        angle2 = 1.0

        try:
            while rclpy.ok():
                rclpy.spin_once(self)
                self.urdf = URDF.from_xml_string(self.robot_description)

                # Extract joint names
                self.joint_names = [joint.name for joint in self.urdf.joints]

                # now we want to publish estimated state messages
                est_state = oem.EstimatedState()
                est_state.joint_names = self.joint_names
                est_state.q_joints = [sin(angle)] * len(self.joint_names)
                est_state.q_base = [sin(angle), cos(angle), sin(angle), 0.0, 0.0, 0.0, 1.0]
                est_state.base_link_name = "pelvis"  # base of the go2, pelvis for g1

                self.est_state_pub.publish(est_state)

                angle += 0.01

                if not self.robot_description2 == "":
                    # self.get_logger().info("has second robot")
                    self.urdf2 = URDF.from_xml_string(self.robot_description2)

                    # Extract joint names
                    self.joint_names2 = [joint.name for joint in self.urdf2.joints]

                    # now we want to publish estimated state messages
                    est_state2 = oem.EstimatedState()
                    est_state2.joint_names = self.joint_names2
                    est_state2.q_joints = [sin(angle2)] * len(self.joint_names2)
                    est_state2.q_base = [sin(angle2), cos(angle2), sin(angle2), 0.0, 0.0, 0.0, 1.0]
                    est_state2.base_link_name = "base"  # base of the go2, pelvis for g1

                    self.est_state_pub2.publish(est_state2)

                    angle2 += 0.01

                # This will adjust as needed per iteration
                loop_rate.sleep()

        except KeyboardInterrupt:
            pass


def main() -> None:  # noqa: D103
    node = StatePublisher()  # noqa: F841


if __name__ == "__main__":
    main()
