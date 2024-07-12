from typing import List, Optional

from rclpy.executors import MultiThreadedExecutor

from obelisk_py.zoo.robot.hardware.leap_hand.leap_node import ObeliskLeapHand
from obelisk_py.core.utils.ros import spin_obelisk


def main(args: Optional[List] = None) -> None:
    """Main entrypoint."""
    spin_obelisk(args, ObeliskLeapHand, MultiThreadedExecutor)


if __name__ == "__main__":
    main()
    