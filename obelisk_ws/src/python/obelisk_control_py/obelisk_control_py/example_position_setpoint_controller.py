from typing import List, Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor

from obelisk_py.zoo.control.example.example_position_setpoint_controller import ExamplePositionSetpointController


def main(args: Optional[List] = None) -> None:
    """Main entrypoint."""
    rclpy.init(args=args)
    example_position_setpoint_controller = ExamplePositionSetpointController()
    executor = SingleThreadedExecutor()
    executor.add_node(example_position_setpoint_controller)
    executor.spin()
    example_position_setpoint_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
