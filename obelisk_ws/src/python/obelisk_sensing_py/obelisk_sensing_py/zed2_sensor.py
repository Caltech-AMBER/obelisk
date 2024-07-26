from typing import List, Optional

from rclpy.executors import MultiThreadedExecutor

from obelisk_py.core.utils.ros import spin_obelisk
from obelisk_py.zoo.sensing.zed2 import ObeliskZed2Sensors


def main(args: Optional[List] = None) -> None:
    """Main entrypoint."""
    spin_obelisk(args, ObeliskZed2Sensors, MultiThreadedExecutor)


if __name__ == "__main__":
    main()
