from typing import List, Optional

from rclpy.executors import SingleThreadedExecutor

from obelisk_py.core.utils.ros import spin_obelisk
from obelisk_py.zoo.control.example.example_position_setpoint_controller import ExamplePositionSetpointController
from obelisk_py.zoo.control.example.d1_controller import D1Controller

def main(args: Optional[List] = None) -> None:
    """Main entrypoint."""
    spin_obelisk(args, D1Controller, SingleThreadedExecutor)

if __name__ == "__main__":
    main()