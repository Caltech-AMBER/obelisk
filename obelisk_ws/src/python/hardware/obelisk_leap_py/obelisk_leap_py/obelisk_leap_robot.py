from typing import List, Optional

from rclpy.executors import SingleThreadedExecutor

from obelisk_py.core.utils.ros import spin_obelisk
from obelisk_py.zoo.hardware.robots.leap.leap_hand_interface import ObeliskLeapRobot


def main(args: Optional[List] = None) -> None:
    """Main entrypoint."""
    spin_obelisk(args, ObeliskLeapRobot, SingleThreadedExecutor)


if __name__ == "__main__":
    main()
