"""Worked example: a minimal ObeliskNode subclass with no controller/estimator/robot role.

Demonstrates that you can subclass ``ObeliskNode`` directly — the framework's role classes
(`ObeliskController`, `ObeliskEstimator`, `ObeliskRobot`, `ObeliskSensor`) are not required for
launching a node through the Obelisk launch pipeline.

Launched by ``obelisk_ws/src/obelisk_ros/config/dummy_minimal.yaml`` under the new top-level
``nodes:`` section.
"""

from typing import List, Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Int32

from obelisk_py.core.node import ObeliskNode


class HeartbeatNode(ObeliskNode):
    """Plain ObeliskNode subclass — no role inheritance — that publishes a counter on a timer."""

    PUB_KEY = "pub_heartbeat"
    TIMER_KEY = "timer_heartbeat"

    def __init__(self) -> None:
        super().__init__("heartbeat_node")
        self._tick = 0
        self.register_obk_publisher(key=self.PUB_KEY, msg_type=Int32)
        self.register_obk_timer(key=self.TIMER_KEY, callback=self._publish_tick)

    def _publish_tick(self) -> None:
        msg = Int32()
        msg.data = self._tick
        self.obk_publishers[self.PUB_KEY].publish(msg)
        self._tick += 1


def main(args: Optional[List] = None) -> None:
    """Spin a HeartbeatNode under a single-threaded executor."""
    rclpy.init(args=args)
    node = HeartbeatNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
