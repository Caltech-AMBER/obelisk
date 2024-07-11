from typing import List, Optional

from rclpy.executors import SingleThreadedExecutor

from obelisk_py.core.utils.ros import spin_obelisk
from obelisk_py.zoo.estimation.jointencoders_passthrough_estimator import JointEncodersPassthroughEstimator


def main(args: Optional[List] = None) -> None:
    """Main entrypoint."""
    spin_obelisk(args, JointEncodersPassthroughEstimator, SingleThreadedExecutor)


if __name__ == "__main__":
    main()
