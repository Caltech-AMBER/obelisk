from typing import List, Optional, Union

import rclpy
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from obelisk_py.core.node import ObeliskNode


def spin_obelisk(
    args: Optional[List],
    node_type: ObeliskNode,
    executor_type: Union[SingleThreadedExecutor, MultiThreadedExecutor],
) -> None:
    """Spin an Obelisk node.

    Parameters:
        args: Command-line arguments.
        node_type: Obelisk node type to spin.
        executor_type: Executor type to use.
    """
    rclpy.init(args=args)
    node = node_type()
    executor = executor_type()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()
