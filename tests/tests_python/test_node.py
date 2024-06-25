from typing import Any, Generator

import obelisk_control_msgs.msg as ocm
import obelisk_estimator_msgs.msg as oem
import obelisk_sensor_msgs.msg as osm
import pytest
from std_msgs.msg import String

from obelisk_py.exceptions import ObeliskMsgError
from obelisk_py.node import ObeliskNode
from obelisk_py.utils import get_classes_in_module

# ##### #
# SETUP #
# ##### #


@pytest.fixture
def test_node(ros_context: Any) -> Generator[ObeliskNode, None, None]:
    """Fixture for the ObeliskNode class."""
    node = ObeliskNode("test_node_exclusivity")
    yield node
    node.destroy_node()


# ##### #
# TESTS #
# ##### #


def test_obelisk_msg_publishing(test_node: ObeliskNode) -> None:
    """Test that all messages in ocm, oem, and osm can be published and subscribed to from an ObeliskNode."""
    for module in [ocm, oem, osm]:
        for msg_type in get_classes_in_module(module):
            test_node.create_publisher(msg_type, f"{msg_type.__name__}_pub", 10)
            test_node.create_subscription(msg_type, f"{msg_type.__name__}_sub", lambda msg: None, 10)


def test_non_obelisk_msg_publishing(test_node: ObeliskNode) -> None:
    """Test that non-obelisk messages cannot be published and subscribed to from an ObeliskNode."""
    with pytest.raises(ObeliskMsgError):
        test_node.create_publisher(String, "/obelisk/test/string", 10)


def test_non_obelisk_msg_publishing_with_flag(test_node: ObeliskNode) -> None:
    """Test that non-obelisk messages can be published and subscribed to from an ObeliskNode when non_obelisk=True."""
    try:
        test_node.create_publisher(String, "/obelisk/test/string_pub", 10, non_obelisk=True)
        test_node.create_subscription(String, "/obelisk/test/string_sub", lambda msg: None, 10, non_obelisk=True)
    except ObeliskMsgError as e:
        pytest.fail(f"Test raised ObeliskMsgError unexpectedly when non_obelisk=True: {e}")
