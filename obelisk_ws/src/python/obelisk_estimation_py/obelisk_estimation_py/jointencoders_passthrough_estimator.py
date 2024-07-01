from typing import List, Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor

from obelisk_py.zoo.estimation.jointencoders_passthrough_estimator import JointEncodersPassthroughEstimator


def main(args: Optional[List] = None) -> None:
    """Main entrypoint."""
    rclpy.init(args=args)
    passthrough_estimator = JointEncodersPassthroughEstimator()
    executor = SingleThreadedExecutor()
    executor.add_node(passthrough_estimator)
    executor.spin()
    passthrough_estimator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
