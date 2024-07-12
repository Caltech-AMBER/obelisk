import obelisk_sensor_msgs.msg as osm

import os
import sys
import math

file_dir = os.path.dirname(__file__)
sys.path.append(file_dir)

import dxl_motor_helper as dxl

from obelisk_py.core.robot import ObeliskRobot
from obelisk_py.core.obelisk_typing import ObeliskControlMsg


class ObeliskLeapHand(ObeliskRobot):

    N_MOTORS = 16

    def __init__(self, node_name):
        super().__init__(node_name)
        dxl.setup()
        dxl.sync_PID(range(self.N_MOTORS))

    @staticmethod
    def radians_to_dxl_pos(radians: float) -> int:
        return int(radians / (2 * math.pi) * 4095)

    def apply_control(self, control_msg: ObeliskControlMsg) -> None:
        for id in range(self.N_MOTORS):
            dxl.write_pos(ObeliskLeapHand.radians_to_dxl_pos(control_msg.u[id]), id=id)