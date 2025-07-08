from typing import List, Optional

from rclpy.executors import SingleThreadedExecutor

from obelisk_py.core.utils.ros import spin_obelisk
from obelisk_py.zoo.estimation.d1_estimator import D1Estimator


def main(args: Optional[List] = None) -> None:
    """Main entrypoint."""
    spin_obelisk(args, D1Estimator, SingleThreadedExecutor)


if __name__ == "__main__":
    main()
