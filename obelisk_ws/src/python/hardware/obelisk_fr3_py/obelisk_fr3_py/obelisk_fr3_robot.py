from typing import List, Optional

from rclpy.executors import SingleThreadedExecutor

from obelisk_py.core.utils.ros import spin_obelisk
from obelisk_py.zoo.hardware.robots.fr3.fr3_interface import ObeliskFR3Robot


def main(args: Optional[List] = None) -> None:
    """Main entrypoint."""
    spin_obelisk(args, ObeliskFR3Robot, SingleThreadedExecutor)


if __name__ == "__main__":
    main()
