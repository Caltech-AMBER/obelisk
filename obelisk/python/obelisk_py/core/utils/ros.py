from typing import List, Optional, Type, Union  # noqa: I001

import rclpy
from rclpy.executors import (
    ExternalShutdownException,
    MultiThreadedExecutor,
    SingleThreadedExecutor,
)

from obelisk_py.core.node import ObeliskNode


def spin_obelisk(
    args: Optional[List],
    node_type: Type[ObeliskNode],
    executor_type: Union[Type[SingleThreadedExecutor], Type[MultiThreadedExecutor]],
    node_name: str = "obelisk_node",
    node_kwargs: Optional[dict] = None,
) -> None:
    """Spin an Obelisk node.

    Parameters:
        args: Command-line arguments.
        node_type: Obelisk node type to spin.
        executor_type: Executor type to use.
        node_kwargs: Keyword arguments to pass to the node
    """
    # rclpy may already be initialized when the global_state node has been launched
    if not rclpy.ok():  # check if rclpy is not initialized
        rclpy.init(args=args)
    node = node_type(node_name=node_name, **(node_kwargs or {}))
    executor = executor_type()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():  # Only shutdown if context is still valid
            rclpy.shutdown()
