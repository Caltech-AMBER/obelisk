import sys

import numpy as np
from dynamixel_sdk import *

PACKET_HANDLER = PacketHandler(protocol_version=2.0)
PORT_HANDLER = PortHandler(port_name='/dev/ttyUSB0')

BAUDRATE = 4000000

class MSG_LENS:
    TOGGLE_TORQUE = 1
    GOAL_POSITION = 4
    PRESENT_POSITION = 4

class ADDR:
    TOGGLE_TORQUE = 64
    GOAL_POSITION = 116
    PRESENT_POSITION = 132
    KP = 84
    KI = 82
    KD = 80


enabled = set()

MIN_POS = 0
MAX_POS = 4095

# Initialize GroupBulkWrite instance
groupBulkWrite = GroupBulkWrite(PORT_HANDLER, PACKET_HANDLER)

# Initialize GroupBulkRead instace for Present Position
groupBulkRead = GroupBulkRead(PORT_HANDLER, PACKET_HANDLER)

def setup(n: int):
    if PORT_HANDLER.openPort():
        print(f"Port {PORT_HANDLER.port_name} opened")
    else:
        print("Failed to open the port")
        sys.exit()
    if PORT_HANDLER.setBaudRate(BAUDRATE):
        print(f"Baudrate set to {PORT_HANDLER.baudrate}")
    else:
        print("Failed to change the baudrate")
        sys.exit()
    for i in range(n):
        if read_enable_status(i):
            enabled.add(i)

def shutdown():
    PORT_HANDLER.closePort()

def enable_motor(id) -> None:
    if id not in enabled:
        enabled.add(id)
    enable_torque(id)
    print(f"Motor {id} enabled")

def disable_motor(id) -> None:
    if id in enabled:
        enabled.remove(id)
    disable_torque(id)
    print(f"Motor {id} disabled")

def read_enable_status(id) -> bool:
    dxl_enabled, dxl_comm_result, dxl_error = PACKET_HANDLER.read1ByteTxRx(PORT_HANDLER, id, ADDR.TOGGLE_TORQUE)
    comm_error_check(dxl_comm_result, dxl_error, id)
    return dxl_enabled

def enable_torque(id) -> None:
    TORQUE_ENABLE = 1
    dxl_comm_result, dxl_error = PACKET_HANDLER.write1ByteTxRx(PORT_HANDLER, id, ADDR.TOGGLE_TORQUE, TORQUE_ENABLE)
    comm_error_check(dxl_comm_result, dxl_error, id)

def disable_torque(id) -> None:
    TORQUE_DISABLE = 0
    dxl_comm_result, dxl_error = PACKET_HANDLER.write1ByteTxRx(PORT_HANDLER, id, ADDR.TOGGLE_TORQUE, TORQUE_DISABLE)
    comm_error_check(dxl_comm_result, dxl_error, id)

def write_pos(pos, id) -> None:
    enable_motor(id)
    dxl_comm_result, dxl_error = PACKET_HANDLER.write4ByteTxRx(PORT_HANDLER, id, ADDR.GOAL_POSITION, np.clip(pos, MIN_POS, MAX_POS))
    comm_error_check(dxl_comm_result, dxl_error, id)

def read_pos(id) -> int:
    enable_motor(id)
    dxl_present_position, dxl_comm_result, dxl_error = PACKET_HANDLER.read4ByteTxRx(PORT_HANDLER, id, ADDR.PRESENT_POSITION)
    comm_error_check(dxl_comm_result, dxl_error, id)
    return dxl_present_position

def comm_error_check(dxl_comm_result, dxl_error, id):
    error = True
    COMM_SUCCESS = 0
    if dxl_comm_result != COMM_SUCCESS:
        print(PACKET_HANDLER.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print(PACKET_HANDLER.getRxPacketError(dxl_error))
    else:
        error = False
    if error:
        print(f"Error in Dynamixel ID: {id}")
        shutdown()
        sys.exit()

def sync_PID(motors, Kp=500, Ki=10, Kd=50):
    for motor in motors:
        enable_motor(motor)
        dxl_comm_result, dxl_error = PACKET_HANDLER.write2ByteTxRx(PORT_HANDLER, motor, ADDR.KP, Kp)
        comm_error_check(dxl_comm_result, dxl_error, motor)
        dxl_comm_result, dxl_error = PACKET_HANDLER.write2ByteTxRx(PORT_HANDLER, motor, ADDR.KI, Ki)
        comm_error_check(dxl_comm_result, dxl_error, motor)
        dxl_comm_result, dxl_error = PACKET_HANDLER.write2ByteTxRx(PORT_HANDLER, motor, ADDR.KD, Kd)
        comm_error_check(dxl_comm_result, dxl_error, motor)