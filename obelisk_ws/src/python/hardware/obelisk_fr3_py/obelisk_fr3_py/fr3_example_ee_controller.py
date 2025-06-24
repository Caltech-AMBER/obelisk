from typing import List, Optional

from rclpy.executors import SingleThreadedExecutor

from obelisk_py.core.utils.ros import spin_obelisk
from obelisk_py.zoo.control.example.fr3_example_ee_controller import FR3ExampleEEController


def main(args: Optional[List] = None) -> None:
    """Main entrypoint."""
    spin_obelisk(args, FR3ExampleEEController, SingleThreadedExecutor)


if __name__ == "__main__":
    main()
