import math
from typing import Type

from dynamixel_sdk import DXL_HIBYTE, DXL_HIWORD, DXL_LOBYTE, DXL_LOWORD
from obelisk_control_msgs.msg import PositionSetpoint
from obelisk_sensor_msgs.msg import ObkJointEncoders
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

import obelisk_py.zoo.hardware.robots.leap.dxl_motor_helper as dxl
from obelisk_py.core.robot import ObeliskRobot


class ObeliskLeapRobot(ObeliskRobot):
    """This class represents the Obelisk Leap Hand robot."""

    N_MOTORS = 16

    def __init__(self, node_name: str) -> None:
        """Initialize the Obelisk Leap Hand robot."""
        super().__init__(node_name, PositionSetpoint)
        self.register_obk_publisher(
            "pub_sensor_setting",
            ObkJointEncoders,
            key="pub_sensor",
        )
        self.register_obk_timer(
            "timer_sensor_setting",
            self.get_state,
            key="timer_sensor",
        )
        dxl.setup(self.N_MOTORS)
        dxl.sync_pid(range(self.N_MOTORS))
        self.yhat = [0] * self.N_MOTORS

    @staticmethod
    def _radians_to_dxl_pos(radians: float) -> int:
        return int((radians + math.pi) / (2 * math.pi) * (dxl.MAX_POS - dxl.MIN_POS))

    @staticmethod
    def _dxl_pos_to_radians(dxl_pos: int) -> float:
        return (dxl_pos / (dxl.MAX_POS - dxl.MIN_POS)) * 2 * math.pi - math.pi

    def apply_control(self, control_msg: Type) -> None:
        """Apply the control message to the robot."""
        for i in range(self.N_MOTORS):
            val = self._radians_to_dxl_pos(control_msg.q_des[i])
            arr = [
                DXL_LOBYTE(DXL_LOWORD(val)),
                DXL_HIBYTE(DXL_LOWORD(val)),
                DXL_LOBYTE(DXL_HIWORD(val)),
                DXL_HIBYTE(DXL_HIWORD(val)),
            ]
            if not dxl.BULK_WRITER.addParam(i, dxl.MsgAddrs.GOAL_POSITION, dxl.MsgLens.GOAL_POSITION, arr):
                print(f"Motor {i}: failed to add param")
        dxl.BULK_WRITER.txPacket()
        dxl.BULK_WRITER.clearParam()

    def get_state(self) -> ObkJointEncoders:
        """Read the joint encoders and publish a sensor message."""
        joint_encoders = ObkJointEncoders()
        for i in range(self.N_MOTORS):
            dxl.BULK_READER.addParam(i, dxl.MsgAddrs.PRESENT_POSITION, dxl.MsgLens.PRESENT_POSITION)
        dxl.BULK_READER.txRxPacket()
        for i in range(self.N_MOTORS):
            position = dxl.BULK_READER.getData(i, dxl.MsgAddrs.PRESENT_POSITION, dxl.MsgLens.PRESENT_POSITION)
            joint_encoders.joint_pos.append(self._dxl_pos_to_radians(position))
        self.obk_publishers["pub_sensor"].publish(joint_encoders)
        return joint_encoders

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Shutdown the LEAP Hand."""
        super().on_shutdown(state)
        dxl.shutdown(self.N_MOTORS)
        return TransitionCallbackReturn.SUCCESS
