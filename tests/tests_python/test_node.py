import obelisk_control_msgs.msg
import pytest
import rclpy
from std_msgs.msg import String

from obelisk_py.exceptions import ObeliskMsgError
from obelisk_py.internal_utils import get_classes_in_module
from obelisk_py.node import ObeliskNode


def test_obelisk_node_exclusivity() -> None:
    """Test that the ObeliskNode class can only publish Obelisk messages."""
    # setup
    rclpy.init()
    node = ObeliskNode("test_node_exclusivity")

    # test that the ObeliskNode class can create publishers/subscribers for all Obelisk messages
    try:
        for msg_type in get_classes_in_module(obelisk_control_msgs.msg):
            node.create_publisher(msg_type, msg_type.__name__ + "_pub", 10)
            node.create_subscription(msg_type, msg_type.__name__ + "_sub", lambda msg: None, 10)
        # for msg_type in get_classes_in_module(obelisk_estimator_msgs.msg):
        #     node.create_publisher(msg_type, msg_type.__name__ + "_pub", 10)
        #     node.create_subscription(msg_type, msg_type.__name__ + "_sub", lambda msg: None, 10)
        # for msg_type in get_classes_in_module(obelisk_sensor_msgs.msg):
        #     node.create_publisher(msg_type, msg_type.__name__ + "_pub", 10)
        #     node.create_subscription(msg_type, msg_type.__name__ + "_sub", lambda msg: None, 10)

    except ObeliskMsgError as e:
        node.destroy_node()
        pytest.fail(f"ObeliskNode.create_publisher() raised ObeliskMsgError unexpectedly: {e}")

    # test that the ObeliskNode class cannot create publishers/subscribers for other messages
    with pytest.raises(ObeliskMsgError):
        node.create_publisher(String, "string", 10)

    node.destroy_node()
    rclpy.shutdown()
