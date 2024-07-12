import dxl_motor_helper as dxl

dxl.setup(16)

for i in range(16):
    dxl.disable_motor(i)

dxl.shutdown()
