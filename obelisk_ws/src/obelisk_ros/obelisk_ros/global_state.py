from typing import List, Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.lifecycle import LifecycleNode


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
    rclpy.init(args=args)
    obelisk_mujoco_robot = GlobalStateNode()
    executor = SingleThreadedExecutor()
    executor.add_node(obelisk_mujoco_robot)
    executor.spin()
    obelisk_mujoco_robot.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
