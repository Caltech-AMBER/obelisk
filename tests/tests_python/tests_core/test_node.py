from typing import Any, Callable, Generator

import obelisk_control_msgs.msg as ocm
import pytest
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.lifecycle.node import TransitionCallbackReturn
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.timer import Timer
from std_msgs.msg import String

from obelisk_py.core.exceptions import ObeliskMsgError
from obelisk_py.core.node import ObeliskNode

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


def test_node_initialization(test_node: ObeliskNode) -> None:
    """Test the initialization of the ObeliskNode.

    This test checks if the node is properly initialized with the correct name and if the callback_group_settings
    parameter is declared.

    Parameters:
        test_node: An instance of ObeliskNode.
    """
    assert test_node.get_name() == "test_node_exclusivity"
    assert test_node.has_parameter("callback_group_settings")


def test_register_obk_publisher(test_node: ObeliskNode) -> None:
    """Test the registration of an Obelisk publisher.

    This test verifies that a publisher can be registered with the node and that the appropriate ROS parameter is
    declared.

    Parameters:
        test_node: An instance of ObeliskNode.
    """
    test_node.register_obk_publisher("test_pub_param", key="test_pub", msg_type=String)
    assert test_node.has_parameter("test_pub_param")


def test_register_obk_subscription(test_node: ObeliskNode) -> None:
    """Test the registration of an Obelisk subscription.

    This test verifies that a subscription can be registered with the node and that the appropriate ROS parameter is
    declared.

    Parameters:
        test_node: An instance of ObeliskNode.
    """

    def callback(msg: String) -> None:
        pass

    test_node.register_obk_subscription("test_sub_param", callback, key="test_sub", msg_type=String)
    assert test_node.has_parameter("test_sub_param")


def test_register_obk_timer(test_node: ObeliskNode) -> None:
    """Test the registration of an Obelisk timer.

    This test verifies that a timer can be registered with the node and that the appropriate ROS parameter is declared.

    Parameters:
        test_node: An instance of ObeliskNode.
    """

    def callback() -> None:
        pass

    test_node.register_obk_timer("test_timer_param", callback, key="test_timer")
    assert test_node.has_parameter("test_timer_param")


def test_create_publisher(test_node: ObeliskNode) -> None:
    """Test the creation of a publisher.

    This test verifies that a publisher can be created with an Obelisk message type and that an ObeliskMsgError is
    raised for non-Obelisk message types.

    Parameters:
        test_node: An instance of ObeliskNode.
    """
    pub = test_node.create_publisher(ocm.PositionSetpoint, "test_topic", 10)
    assert isinstance(pub, Publisher)

    with pytest.raises(ObeliskMsgError):
        test_node.create_publisher(String, "test_topic", 10)


def test_create_subscription(test_node: ObeliskNode) -> None:
    """Test the creation of a subscription.

    This test verifies that a subscription can be created with an Obelisk message type and that an ObeliskMsgError is
    raised for non-Obelisk message types.

    Parameters:
        test_node: An instance of ObeliskNode.
    """

    def callback(msg: ocm.PositionSetpoint) -> None:
        pass

    sub = test_node.create_subscription(ocm.PositionSetpoint, "test_topic", callback, 10)
    assert isinstance(sub, Subscription)

    with pytest.raises(ObeliskMsgError):
        test_node.create_subscription(String, "test_topic", callback, 10)


def test_lifecycle_callbacks(test_node: ObeliskNode) -> None:
    """Test the lifecycle callbacks of the ObeliskNode.

    This test verifies that the lifecycle callbacks (on_configure, on_activate, on_deactivate, on_cleanup, on_shutdown)
    return the expected TransitionCallbackReturn.

    Parameters:
        test_node: An instance of ObeliskNode.
    """
    assert test_node.on_configure(None) == TransitionCallbackReturn.SUCCESS
    assert test_node.on_activate(None) == TransitionCallbackReturn.SUCCESS
    assert test_node.on_deactivate(None) == TransitionCallbackReturn.SUCCESS
    assert test_node.on_cleanup(None) == TransitionCallbackReturn.SUCCESS
    assert test_node.on_shutdown(None) == TransitionCallbackReturn.SUCCESS


def test_callback_group_creation(test_node: ObeliskNode, set_node_parameters: Callable) -> None:
    """Test the creation of callback groups.

    This test verifies that callback groups can be created from a configuration string.

    Parameters:
        test_node: An instance of ObeliskNode.
        set_node_parameters: A fixture to set node parameters.
    """
    set_node_parameters(
        test_node, {"callback_group_settings": "group1:MutuallyExclusiveCallbackGroup,group2:ReentrantCallbackGroup"}
    )
    test_node.on_configure(None)
    assert hasattr(test_node, "group1")
    assert isinstance(test_node.group1, MutuallyExclusiveCallbackGroup)
    assert hasattr(test_node, "group2")
    assert isinstance(test_node.group2, ReentrantCallbackGroup)


def test_publisher_creation_from_config(test_node: ObeliskNode, set_node_parameters: Callable) -> None:
    """Test the creation of a publisher from a configuration string.

    This test verifies that a publisher can be created from a configuration string.

    Parameters:
        test_node: An instance of ObeliskNode.
        set_node_parameters: A fixture to set node parameters.
    """
    test_node.register_obk_publisher("test_pub_param", key="test_pub", msg_type=ocm.PositionSetpoint)
    set_node_parameters(test_node, {"test_pub_param": "topic:/test_topic,msg_type:PositionSetpoint,history_depth:10"})
    test_node.on_configure(None)

    assert "test_pub" in test_node.obk_publishers
    assert isinstance(test_node.obk_publishers["test_pub"], Publisher)


def test_subscription_creation_from_config(test_node: ObeliskNode, set_node_parameters: Callable) -> None:
    """Test the creation of a subscription from a configuration string.

    This test verifies that a subscription can be created from a configuration string.

    Parameters:
        test_node: An instance of ObeliskNode.
        set_node_parameters: A fixture to set node parameters.
    """

    def callback(msg: ocm.PositionSetpoint) -> None:
        pass

    test_node.register_obk_subscription("test_sub_param", callback, key="test_sub", msg_type=ocm.PositionSetpoint)
    set_node_parameters(test_node, {"test_sub_param": "topic:/test_topic,msg_type:PositionSetpoint,history_depth:10"})
    test_node.on_configure(None)

    assert "test_sub" in test_node.obk_subscriptions
    assert isinstance(test_node.obk_subscriptions["test_sub"], Subscription)


def test_timer_creation_from_config(test_node: ObeliskNode, set_node_parameters: Callable) -> None:
    """Test the creation of a timer from a configuration string.

    This test verifies that a timer can be created from a configuration string.

    Parameters:
        test_node: An instance of ObeliskNode.
        set_node_parameters: A fixture to set node parameters.
    """

    def callback() -> None:
        pass

    test_node.register_obk_timer("test_timer_param", callback, key="test_timer")
    set_node_parameters(test_node, {"test_timer_param": "timer_period_sec:1.0"})
    test_node.on_configure(None)

    assert "test_timer" in test_node.obk_timers
    assert isinstance(test_node.obk_timers["test_timer"], Timer)
