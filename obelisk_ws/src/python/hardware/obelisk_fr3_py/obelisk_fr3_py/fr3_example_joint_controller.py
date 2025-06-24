from typing import List, Optional

from rclpy.executors import SingleThreadedExecutor

from obelisk_py.core.utils.ros import spin_obelisk
from obelisk_py.zoo.control.example.fr3_example_joint_controller import FR3ExampleJointController


def main(args: Optional[List] = None) -> None:
    """Main entrypoint."""
    spin_obelisk(args, FR3ExampleJointController, SingleThreadedExecutor)


if __name__ == "__main__":
    main()
