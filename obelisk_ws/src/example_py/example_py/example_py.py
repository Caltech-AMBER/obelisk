from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from typing_extensions import Self


class MinimalPublisher(Node):
    """Minimal publisher class.

    See: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
    """

    def __init__(self: Self) -> None:
        """Initialize the node and create a publisher."""
        super().__init__("minimal_publisher")
        self.publisher_ = self.create_publisher(String, "topic", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self: Self) -> None:
        """Publish a message on the topic."""
        msg = String()
        msg.data = "Hello World: %d" % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args: Any = None) -> None:
    """Run the node."""
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
