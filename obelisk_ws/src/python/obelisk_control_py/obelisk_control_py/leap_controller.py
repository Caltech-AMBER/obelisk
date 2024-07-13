from typing import List, Optional

from rclpy.executors import SingleThreadedExecutor

from obelisk_py.core.utils.ros import spin_obelisk
from obelisk_py.zoo.control.example.leap_pos_setpoint import LeapPositionSetpointController


def main(args: Optional[List] = None) -> None:
    """Main entrypoint."""
    spin_obelisk(args, LeapPositionSetpointController, SingleThreadedExecutor)


if __name__ == "__main__":
    main()
