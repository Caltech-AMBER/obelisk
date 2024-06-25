from typing import Any, Generator

import obelisk_control_msgs.msg as ocm
import obelisk_estimator_msgs.msg as oem
import obelisk_sensor_msgs.msg as osm
import pytest
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.lifecycle.node import TransitionCallbackReturn
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.timer import Timer
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


def test_parse_config_str() -> None:
    """Test the _parse_config_str method."""
    config_str = "field1:value1,field2:42,field3:3.14"
    field_names, value_names = ObeliskNode._parse_config_str(config_str)
    assert field_names == ["field1", "field2", "field3"]
    assert value_names == ["value1", 42, 3.14]


def test_parse_config_str_empty() -> None:
    """Test the _parse_config_str method with an empty string."""
    config_str = ""
    field_names, value_names = ObeliskNode._parse_config_str(config_str)
    assert field_names == []
    assert value_names == []


def test_check_fields() -> None:
    """Test the _check_fields method."""
    field_names = ["field1", "field2", "field3"]
    required_fields = ["field1", "field2"]
    optional_fields = ["field3", "field4"]
    ObeliskNode._check_fields(field_names, required_fields, optional_fields)


def test_check_fields_missing_required() -> None:
    """Test the _check_fields method with missing required fields."""
    field_names = ["field1", "field3"]
    required_fields = ["field1", "field2"]
    optional_fields = ["field3", "field4"]
    with pytest.raises(AssertionError):
        ObeliskNode._check_fields(field_names, required_fields, optional_fields)


def test_check_values() -> None:
    """Test the _check_values method."""
    value_names = ["value1", "value2"]
    allowable_value_names = ["value1", "value2", "value3"]
    ObeliskNode._check_values(value_names, allowable_value_names)


def test_check_values_invalid() -> None:
    """Test the _check_values method with invalid values."""
    value_names = ["value1", "invalid_value"]
    allowable_value_names = ["value1", "value2", "value3"]
    with pytest.raises(AssertionError):
        ObeliskNode._check_values(value_names, allowable_value_names)


def test_create_callback_groups_from_config_str(test_node: ObeliskNode) -> None:
    """Test creating callback groups from configuration strings."""
    config_strs = ["group1:MutuallyExclusiveCallbackGroup", "group2:ReentrantCallbackGroup"]
    callback_groups = test_node._create_callback_groups_from_config_str(config_strs)
    assert len(callback_groups) == len(config_strs)
    assert isinstance(callback_groups["group1"], MutuallyExclusiveCallbackGroup)
    assert isinstance(callback_groups["group2"], ReentrantCallbackGroup)


def test_create_publisher_from_config_str(test_node: ObeliskNode) -> None:
    """Test creating a publisher from a configuration string."""
    config_str = "msg_type:JointEncoder,topic:/test/create_publisher_from_config_str,history_depth:20"
    publisher = test_node._create_publisher_from_config_str(config_str, "odom")
    assert isinstance(publisher, Publisher)
    assert publisher.topic_name == "/test/create_publisher_from_config_str"


def test_create_subscription_from_config_str(test_node: ObeliskNode) -> None:
    """Test creating a subscription from a configuration string."""

    def dummy_callback(_: osm.JointEncoder) -> None:
        pass

    test_node.dummy_callback = dummy_callback
    config_str = (
        "msg_type:JointEncoder,topic:/test/create_subscription_from_config_str,callback:dummy_callback,history_depth:20"
    )
    subscription = test_node._create_subscription_from_config_str(config_str, "odom")
    assert isinstance(subscription, Subscription)
    assert subscription.topic_name == "/test/create_subscription_from_config_str"


def test_create_timer_from_config_str(test_node: ObeliskNode) -> None:
    """Test creating a timer from a configuration string."""

    def dummy_timer_callback() -> None:
        pass

    test_node.dummy_timer_callback = dummy_timer_callback
    period = 0.1
    config_str = f"timer_period_sec:{period},callback:dummy_timer_callback"
    timer = test_node._create_timer_from_config_str(config_str)
    assert isinstance(timer, Timer)
    assert timer.timer_period_ns == period * 1e9


def test_on_configure(test_node: ObeliskNode) -> None:
    """Test the on_configure method."""
    test_node.callback_group_config_strs = ["group1:MutuallyExclusiveCallbackGroup"]
    test_node.group1 = MutuallyExclusiveCallbackGroup()
    result = test_node.on_configure(None)
    assert result == TransitionCallbackReturn.SUCCESS
    assert hasattr(test_node, "group1")
    assert isinstance(test_node.group1, MutuallyExclusiveCallbackGroup)


def test_on_cleanup(test_node: ObeliskNode) -> None:
    """Test the on_cleanup method."""
    test_node.callback_group_config_strs = ["group1:MutuallyExclusiveCallbackGroup"]
    test_node.group1 = MutuallyExclusiveCallbackGroup()
    result = test_node.on_cleanup(None)
    assert result == TransitionCallbackReturn.SUCCESS
    assert not hasattr(test_node, "group1")
    assert not hasattr(test_node, "callback_group_config_strs")


def test_on_shutdown(test_node: ObeliskNode) -> None:
    """Test the on_shutdown method."""
    test_node.callback_group_config_strs = ["group1:MutuallyExclusiveCallbackGroup"]
    test_node.group1 = MutuallyExclusiveCallbackGroup()
    result = test_node.on_shutdown(None)
    assert result == TransitionCallbackReturn.SUCCESS
    assert not hasattr(test_node, "group1")
    assert not hasattr(test_node, "callback_group_config_strs")


def test_non_obelisk_msg_publishing_with_flag_publisher(test_node: ObeliskNode) -> None:
    """Test that non-obelisk messages can be published from an ObeliskNode when non_obelisk=True."""
    publisher = test_node.create_publisher(String, "/test/string_pub", 10, non_obelisk=True)
    assert isinstance(publisher, Publisher)
    assert publisher.msg_type == String


def test_non_obelisk_msg_publishing_with_flag_subscription(test_node: ObeliskNode) -> None:
    """Test that non-obelisk messages can be subscribed to from an ObeliskNode when non_obelisk=True."""

    def dummy_callback(_: String) -> None:
        pass

    subscription = test_node.create_subscription(String, "/test/string_sub", dummy_callback, 10, non_obelisk=True)
    assert isinstance(subscription, Subscription)
    assert subscription.msg_type == String
