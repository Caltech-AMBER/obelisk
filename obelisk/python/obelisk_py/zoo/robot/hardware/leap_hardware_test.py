import math
import time

import dxl_motor_helper as dxl

dxl.setup()
init_pos = dxl.read_pos(id=1)



while 1:
    dxl.write_pos(init_pos + int(math.sin(time.time() * 3) * 200), id=1)
    dxl.write_pos(init_pos + int(math.sin(time.time() * 5) * 200), id=2)
    
# test_motor.disable_torque()
dxl.shutdown()