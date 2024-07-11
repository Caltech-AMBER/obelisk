from typing import List, Optional

from rclpy.executors import SingleThreadedExecutor

from obelisk_py.core.utils.ros import spin_obelisk
from obelisk_py.zoo.control.example.example_position_setpoint_controller import ExamplePositionSetpointController


def main(args: Optional[List] = None) -> None:
    """Main entrypoint."""
    spin_obelisk(args, ExamplePositionSetpointController, SingleThreadedExecutor)


if __name__ == "__main__":
    main()
