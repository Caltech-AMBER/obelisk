import sys
from typing import Iterable

from dynamixel_sdk import (
    GroupBulkRead,
    GroupBulkWrite,
    PacketHandler,
    PortHandler,
)

PACKET_HANDLER = PacketHandler(protocol_version=2.0)
PORT_HANDLER = PortHandler(port_name="/dev/ttyUSB0")
BULK_WRITER = GroupBulkWrite(PORT_HANDLER, PACKET_HANDLER)
BULK_READER = GroupBulkRead(PORT_HANDLER, PACKET_HANDLER)

MIN_POS = 0
MAX_POS = 4095
BAUDRATE = 4000000
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0


class MsgLens:
    """Message lengths for different types of messages."""

    TOGGLE_TORQUE = 1
    GOAL_POSITION = 4
    PRESENT_POSITION = 4


class MsgAddrs:
    """Addresses for different types of messages."""

    TOGGLE_TORQUE = 64
    GOAL_POSITION = 116
    PRESENT_POSITION = 132
    KP = 84
    KI = 82
    KD = 80


def setup(n: int) -> None:
    """Setup the Dynamixel motors.

    Args:
        n: The number of motors to setup (the motor IDs are 0 to n-1).
    """
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
        toggle_motor_torque(i, TORQUE_ENABLE)


def shutdown(n: int) -> None:
    """Shutdown the Dynamixel motors.

    Args:
        n: The number of motors to shutdown (the motor IDs are 0 to n-1).
    """
    for i in range(n):
        toggle_motor_torque(i, TORQUE_DISABLE)
    PORT_HANDLER.closePort()


def toggle_motor_torque(id: int, mode: int) -> None:
    """Enable/disable a motor.

    Args:
        id: The motor ID.
        mode: The mode to set the motor to.
    """
    dxl_comm_result, dxl_error = PACKET_HANDLER.write1ByteTxRx(
        PORT_HANDLER, id, MsgAddrs.TOGGLE_TORQUE, mode
    )
    comm_error_check(dxl_comm_result, dxl_error, id)
    print(f"Motor {id} enabled")


def sync_pid(
    motors: Iterable, kp: int = 500, ki: int = 10, kd: int = 50
) -> None:
    """Write PID values to multiple motors.

    Args:
        motors: The motor IDs to write to.
        kp: The proportional gain.
        ki: The integral gain.Kd
        kd: The derivative gain.
    """
    for motor in motors:
        dxl_comm_result, dxl_error = PACKET_HANDLER.write2ByteTxRx(
            PORT_HANDLER, motor, MsgAddrs.KP, kp
        )
        comm_error_check(dxl_comm_result, dxl_error, motor)
        dxl_comm_result, dxl_error = PACKET_HANDLER.write2ByteTxRx(
            PORT_HANDLER, motor, MsgAddrs.KI, ki
        )
        comm_error_check(dxl_comm_result, dxl_error, motor)
        dxl_comm_result, dxl_error = PACKET_HANDLER.write2ByteTxRx(
            PORT_HANDLER, motor, MsgAddrs.KD, kd
        )
        comm_error_check(dxl_comm_result, dxl_error, motor)


def comm_error_check(dxl_comm_result: int, dxl_error: int, id: int) -> None:
    """Check for communication errors.

    Args:
        dxl_comm_result: The communication result.
        dxl_error: The error code.
        id: The motor ID.
    """
    comm_success = 0
    if dxl_comm_result != comm_success:
        print(PACKET_HANDLER.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print(PACKET_HANDLER.getRxPacketError(dxl_error))
    else:
        return
    print(f"Error in Dynamixel ID: {id}")
