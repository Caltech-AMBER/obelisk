import math
import time

import dxl_motor_helper as dxl

dxl.setup()

for i in range(16):
    dxl.enable_motor(i)
    dxl.disable_motor(i)

dxl.shutdown()