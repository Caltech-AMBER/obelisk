from typing import List, Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.lifecycle import LifecycleNode

from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor, SingleThreadedExecutor

class GlobalStateNode(LifecycleNode):
    """A dummy class whose only purpose is to store the global state of the system.

    In particular, the lifecycle state of this node will be tracked and matched by all other Obelisk nodes. Thus, we can
    configure, activate, deactivate, and shutdown all nodes in the system at once.
    """

    def __init__(self) -> None:
        """Initialize the GlobalStateNode."""
        super().__init__("global_state")


def main(args: Optional[List] = None) -> None:
    """Main entrypoint."""
    # rclpy may already be initialized when the other nodes have been launched
    if not rclpy.ok(): # check if rclpy is not initialized
        rclpy.init(args=args)
    node = GlobalStateNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok(): # Only shutdown if context is still valid
            rclpy.shutdown()

if __name__ == "__main__":
    main()
