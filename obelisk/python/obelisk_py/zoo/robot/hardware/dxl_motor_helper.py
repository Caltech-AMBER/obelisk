import sys

import numpy as np
from dynamixel_sdk import *

PACKET_HANDLER = PacketHandler(protocol_version=2.0)
PORT_HANDLER = PortHandler(port_name='/dev/ttyUSB0')

enabled = set()

MIN_POS = 0
MAX_POS = 4095

def setup():
    baudrate = 4000000
    if PORT_HANDLER.openPort():
        print(f"Port {PORT_HANDLER.port_name} opened")
    else:
        print("Failed to open the port")
        sys.exit()
    if PORT_HANDLER.setBaudRate(baudrate):
        print(f"Baudrate set to {PORT_HANDLER.baudrate}")
    else:
        print("Failed to change the baudrate")
        sys.exit()

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

def enable_torque(id) -> None:
    ADDR_TOGGLE_TORQUE = 64
    TORQUE_ENABLE = 1
    dxl_comm_result, dxl_error = PACKET_HANDLER.write1ByteTxRx(PORT_HANDLER, id, ADDR_TOGGLE_TORQUE, TORQUE_ENABLE)
    comm_error_check(dxl_comm_result, dxl_error, id)

def disable_torque(id) -> None:
    ADDR_TOGGLE_TORQUE = 64
    TORQUE_DISABLE = 0
    dxl_comm_result, dxl_error = PACKET_HANDLER.write1ByteTxRx(PORT_HANDLER, id, ADDR_TOGGLE_TORQUE, TORQUE_DISABLE)
    comm_error_check(dxl_comm_result, dxl_error, id)

def write_pos(pos, id) -> None:
    enable_motor(id)
    ADDR_GOAL_POSITION = 116
    dxl_comm_result, dxl_error = PACKET_HANDLER.write4ByteTxRx(PORT_HANDLER, id, ADDR_GOAL_POSITION, np.clip(pos, MIN_POS, MAX_POS))
    comm_error_check(dxl_comm_result, dxl_error, id)

def read_pos(id) -> int:
    enable_motor(id)
    ADDR_PRESENT_POSITION = 132
    dxl_present_position, dxl_comm_result, dxl_error = PACKET_HANDLER.read4ByteTxRx(PORT_HANDLER, id, ADDR_PRESENT_POSITION)
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

def sync_PID(motors, Kp=400, Ki=10, Kd=50):
    Kp_addr = 84
    Ki_addr = 82
    Kd_addr = 80
    for motor in motors:
        enable_motor(motor)
        dxl_comm_result, dxl_error = PACKET_HANDLER.write2ByteTxRx(PORT_HANDLER, motor, Kp_addr, Kp)
        comm_error_check(dxl_comm_result, dxl_error, motor)
        dxl_comm_result, dxl_error = PACKET_HANDLER.write2ByteTxRx(PORT_HANDLER, motor, Ki_addr, Ki)
        comm_error_check(dxl_comm_result, dxl_error, motor)
        dxl_comm_result, dxl_error = PACKET_HANDLER.write2ByteTxRx(PORT_HANDLER, motor, Kd_addr, Kd)
        comm_error_check(dxl_comm_result, dxl_error, motor)