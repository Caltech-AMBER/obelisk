from typing import List, Optional, Type, Union

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor, SingleThreadedExecutor

from obelisk_py.core.node import ObeliskNode


def spin_obelisk(
    args: Optional[List],
    node_type: Type[ObeliskNode],
    executor_type: Union[Type[SingleThreadedExecutor], Type[MultiThreadedExecutor]],
    node_kwargs: Optional[dict] = None,
) -> None:
    """Spin an Obelisk node.

    Parameters:
        args: Command-line arguments.
        node_type: Obelisk node type to spin.
        executor_type: Executor type to use.
        node_kwargs: Keyword arguments to pass to the node
    """
    rclpy.init(args=args)
    node = node_type(node_name="obelisk_node", **(node_kwargs or {}))
    executor = executor_type()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
