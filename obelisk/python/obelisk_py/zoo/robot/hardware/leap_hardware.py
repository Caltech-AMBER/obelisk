import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState

from leap_msgs.srv import (
    LEAPCurrent,
    LEAPPosition,
    LEAPTorque,
    LEAPVelocity,
)
from .utils.dynamixel_client import (
    dynamixel_cleanup_handler,
    DynamixelClient,
    DynamixelCurReader,
    DynamixelPosReader,
    DynamixelPosVelCurReader,
    DynamixelReader,
    DynamixelVelReader,
)


class LEAPNode(Node):
    """Node for controlling the LEAP hand.

    Modified from https://github.com/leap-hand/LEAP_Hand_API/blob/main/ros_module/leaphand_node.py

    [NOTE] The Dynamixel motors use the convention where pi radians is the resting position. In sim,
    it is convenient to have this be 0, so we must add/subtract pi as needed to convert between them.
    """

    def __init__(self) -> None:
        """Initialize the LEAP hand node."""
        super().__init__("leap_node")

        # ############## #
        # ROS PARAMETERS #
        # ############## #
        # hardware parameters
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 4000000)
        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value

        # gains
        self.declare_parameter("KP", 150.0)  # original: 600.0
        self.declare_parameter("KI", 0.0)  # original: 0.0
        self.declare_parameter("KD", 50.0)  # original: 200.0
        self.KP = self.get_parameter("KP").get_parameter_value().double_value
        self.KI = self.get_parameter("KI").get_parameter_value().double_value
        self.KD = self.get_parameter("KD").get_parameter_value().double_value

        # limits
        self.declare_parameter("curr_lim", 500)  # mA, 350 for lite, 500 for regular hand
        self.curr_lim = self.get_parameter("curr_lim").get_parameter_value().integer_value

        # states
        self.q_des_prev = self.q_des_curr = np.zeros(16)  # neutral position

        # physical configuration - used for gravity compensation
        # [NOTE] currently only uses the cube rotation setup, so doesn't have to consider arm kinematics
        self.declare_parameter("config", "cube")
        self.config = self.get_parameter("config").get_parameter_value().string_value

        # ########################## #
        # PUBLISHERS AND SUBSCRIBERS #
        # ########################## #
        self.leap_position_srv = self.create_service(
            LEAPPosition,
            "/leap_position",
            self.leap_position_callback,
        )
        self.leap_velocity_srv = self.create_service(
            LEAPVelocity,
            "/leap_velocity",
            self.leap_velocity_callback,
        )
        self.leap_current_srv = self.create_service(
            LEAPCurrent,
            "/leap_current",
            self.leap_current_callback,
        )
        self.leap_torque_srv = self.create_service(
            LEAPTorque,
            "/leap_torque",
            self.leap_torque_callback,
        )

        subscriber_cbg = MutuallyExclusiveCallbackGroup()
        self.q_des_subcriber = self.create_subscription(
            JointState,
            "/leap/q_des",
            self.q_des_callback,
            10,
            callback_group=subscriber_cbg,
        )

        # ####################### #
        # CURRENT-TO-TORQUE MODEL #
        # ####################### #
        # [WARNING] probably not well-calibrated
        degree = 8  # we fit a degree 8 polynomial to the motor curve data
        self.cur2torque_features = lambda X: np.vander(X, degree + 1, increasing=True)[:, 1:]
        self.cur2torque_coefs = np.array(
            [
                1.75467424e-02,
                -1.25683563e00,
                1.21950491e01,
                -3.00522833e01,
                3.60879412e01,
                -2.34213875e01,
                7.89234331e00,
                -1.08440513e00,
            ]
        )

        # ########### #
        # MOTOR SETUP #
        # ########### #
        self.motors = motors = [i for i in range(16)]  # 0 to 15
        self.dxl_client = DynamixelClient(motors, self.port, self.baudrate)
        self.dxl_client.connect()

        # enables position-current control mode and the default parameters
        # it commands a position and then caps the current so the motors don't overload
        self.dxl_client.sync_write(motors, np.ones(len(motors)) * 5, 11, 1)
        self.dxl_client.set_torque_enabled(motors, True)

        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.KP, 84, 2)
        # self.dxl_client.sync_write([0, 4, 8], np.ones(3) * (self.KP * 0.75), 84, 2)  # P for side to side should be a bit less

        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.KI, 82, 2)

        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.KD, 80, 2)
        # self.dxl_client.sync_write([0, 4, 8], np.ones(3) * (self.KD * 0.75), 80, 2)  # D for side to side should be a bit less

        self.dxl_client.sync_write(motors, np.ones(len(motors)) * self.curr_lim, 102, 2)  # curr lims
        q_start = np.array([0.5, -0.75, 0.75, 0.25, 0.5, 0, 0.75, 0.25, 0.5, 0.75, 0.75, 0.25, 0.65, 0.9, 0.75, 0.6])
        self.dxl_client.write_desired_pos(self.motors, q_start + np.pi)  # initialize hand in neutralize position

    def q_des_callback(self, msg: JointState) -> None:
        """Writes the desired joint states to the Dynamixel servos."""
        _q_des = msg.position
        self.q_des_prev = self.q_des_curr
        self.q_des_curr = np.array(_q_des) + np.pi  # converting to Dynamixel coords
        self.dxl_client.write_desired_pos(self.motors, self.q_des_curr)

    def leap_position_callback(self, _request, response):
        """Computes the position of the LEAP hand in 0-center coordinates."""
        response.q = (self.dxl_client.read_pos() - np.pi).tolist()
        return response

    def leap_velocity_callback(self, _request, response):
        """Computes the velocity of the LEAP hand."""
        response.dq = self.dxl_client.read_vel().tolist()
        return response

    def leap_current_callback(self, _request, response):
        """Computes the current of the LEAP hand."""
        response.current = self.dxl_client.read_cur().tolist()
        return response

    def leap_torque_callback(self, _request, response):
        """Computes the current of the LEAP hand.

        [WARNING] Probably not well-calibrated!
        """
        curr = self.dxl_client.read_cur() / 1000.0  # (16,) in amps
        curr_signs = np.sign(curr)
        curr_abs = np.abs(curr)
        A = self.cur2torque_features(curr_abs)  # (16, 8) no bias term
        unsigned_torque = A @ self.cur2torque_coefs
        unsigned_torque[unsigned_torque < 0.0]  # no negative torques
        response.torque = (unsigned_torque * curr_signs).tolist()
        return response


def main(args=None):
    rclpy.init(args=args)
    leap_node = LEAPNode()
    rclpy.spin(leap_node)
    dynamixel_cleanup_handler()


if __name__ == "__main__":
    main()
