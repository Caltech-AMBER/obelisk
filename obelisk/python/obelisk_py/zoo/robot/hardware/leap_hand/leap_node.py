import os
import sys
import math

file_dir = os.path.dirname(__file__)
sys.path.append(file_dir)

import dxl_motor_helper as dxl
from dynamixel_sdk import *

from obelisk_py.core.robot import ObeliskRobot
from obelisk_py.core.obelisk_typing import ObeliskControlMsg, ObeliskSensorMsg
from obelisk_sensor_msgs.msg import JointEncoders



class ObeliskLeapHand(ObeliskRobot):

    N_MOTORS = 16

    def __init__(self, node_name):
        super().__init__(node_name)
        # self.register_obk_publisher("pub_joint_encoders", key="pub_joint_encoders", msg_type=JointEncoders)
        dxl.setup(self.N_MOTORS)
        dxl.sync_PID(range(self.N_MOTORS))
        for i in range(self.N_MOTORS):
            dxl.enable_motor(i)


    @staticmethod
    def _radians_to_dxl_pos(radians: float) -> int:
        return int((radians + math.pi) / (2 * math.pi) * 4095)
    
    @staticmethod
    def _dxl_pos_to_radians(dxl_pos: int) -> float:
        return (dxl_pos / 4095) * 2 * math.pi - math.pi


    def apply_control(self, control_msg: ObeliskControlMsg) -> None:
        for i in range(self.N_MOTORS):
            val = self._radians_to_dxl_pos(control_msg.u[i])
            arr = [DXL_LOBYTE(DXL_LOWORD(val)), DXL_HIBYTE(DXL_LOWORD(val)), DXL_LOBYTE(DXL_HIWORD(val)), DXL_HIBYTE(DXL_HIWORD(val))]
            if not dxl.groupBulkWrite.addParam(i, dxl.ADDR.GOAL_POSITION, dxl.MSG_LENS.GOAL_POSITION, arr):
                print("failed to add param")
        dxl.groupBulkWrite.txPacket()
        dxl.groupBulkWrite.clearParam()
        print(self.get_state().y)


    def get_state(self) -> ObeliskSensorMsg:
        joint_encoders = JointEncoders()
        for i in range(self.N_MOTORS):
            dxl.groupBulkRead.addParam(i, dxl.ADDR.PRESENT_POSITION, dxl.MSG_LENS.PRESENT_POSITION)
        dxl.groupBulkRead.txRxPacket()
        for i in range(self.N_MOTORS):
            position = dxl.groupBulkRead.getData(i, dxl.ADDR.PRESENT_POSITION, dxl.MSG_LENS.PRESENT_POSITION)
            joint_encoders.y.append(self._dxl_pos_to_radians(position))
        return joint_encoders