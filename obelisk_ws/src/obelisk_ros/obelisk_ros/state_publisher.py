from math import cos, pi, sin

import rclpy
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from urdf_parser_py.urdf import URDF


class StatePublisher(Node):
    # TODO: Re-write this to be a proper node so that I can declare params properly
    #   Make sure that I can receive the xml correctly
    #   Make sure the joint names string list is correct
    #   Publish the correct joint state message
    #   Verify all the stuff with the fixed frame
    #   Make a good rviz config
    def __init__(self):
        rclpy.init()
        super().__init__("state_publisher")

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, "joint_states", qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        self.declare_parameter("robot_description", "")
        self.robot_description = self.get_parameter("robot_description").get_parameter_value().string_value
        self.get_logger().info("robot: " + self.robot_description)  #  + self.robot_description

        degree = pi / 180.0
        loop_rate = self.create_rate(30)

        # robot state
        tilt = 0.0
        tinc = degree
        swivel = 0.0
        angle = 0.0
        height = 0.0
        hinc = 0.005

        # message declarations
        odom_trans = TransformStamped()
        odom_trans.header.frame_id = "world"
        odom_trans.child_frame_id = "pelvis"
        joint_state = JointState()

        try:
            while rclpy.ok():
                rclpy.spin_once(self)
                self.urdf = URDF.from_xml_string(self.robot_description)

                # Extract joint names
                self.joint_names = [joint.name for joint in self.urdf.joints]

                # update joint_state
                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()
                joint_state.name = self.joint_names
                joint_state.position = [0.0] * len(self.joint_names)

                # update transform
                # (moving in a circle with radius=2)
                odom_trans.header.stamp = now.to_msg()
                odom_trans.transform.translation.x = cos(angle) * 2
                odom_trans.transform.translation.y = sin(angle) * 2
                odom_trans.transform.translation.z = 0.7
                odom_trans.transform.rotation = euler_to_quaternion(0, 0, angle + pi / 2)  # roll,pitch,yaw

                # send the joint state and transform
                self.joint_pub.publish(joint_state)
                self.broadcaster.sendTransform(odom_trans)

                # Create new robot state
                tilt += tinc
                if tilt < -0.5 or tilt > 0.0:
                    tinc *= -1
                height += hinc
                if height > 0.2 or height < 0.0:
                    hinc *= -1
                swivel += degree
                angle += degree / 4

                # This will adjust as needed per iteration
                loop_rate.sleep()

        except KeyboardInterrupt:
            pass


def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
    qy = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2)
    qz = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2)
    qw = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main():
    node = StatePublisher()


if __name__ == "__main__":
    main()
