"""Tests for ObeliskNode (post-config-string-refactor).

These exercise the new ``obelisk_settings`` YAML-string parameter and the simplified
``register_obk_*(key, ...)`` API.
"""

from typing import Any, Callable, Generator

import obelisk_control_msgs.msg as ocm
import pytest
import yaml
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.lifecycle.node import TransitionCallbackReturn
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.timer import Timer
from std_msgs.msg import String

from obelisk_py.core.node import ObeliskNode


@pytest.fixture
def test_node(ros_context: Any) -> Generator[ObeliskNode, None, None]:
    """Fixture for the ObeliskNode class."""
    node = ObeliskNode("test_node_exclusivity")
    yield node
    node.destroy_node()


def _set_obelisk_settings(node: ObeliskNode, settings: dict, set_node_parameters: Callable) -> None:
    """Serialize ``settings`` to YAML and assign it to the node's ``obelisk_settings`` parameter."""
    set_node_parameters(node, {"obelisk_settings": yaml.safe_dump(settings)})


def test_node_initialization(test_node: ObeliskNode) -> None:
    """The node declares the single ``obelisk_settings`` parameter and the user ``params_path`` slot."""
    assert test_node.get_name() == "test_node_exclusivity"
    assert test_node.has_parameter("obelisk_settings")
    assert test_node.has_parameter("params_path")


def test_register_obk_publisher(test_node: ObeliskNode) -> None:
    """register_obk_publisher records the registration without declaring a per-component parameter."""
    test_node.register_obk_publisher(key="test_pub", msg_type=String)
    assert any(s["key"] == "test_pub" for s in test_node._obk_pub_settings)
    # No legacy *_setting parameter should be declared anymore.
    assert not test_node.has_parameter("test_pub_setting")


def test_register_obk_subscription(test_node: ObeliskNode) -> None:
    """register_obk_subscription stores the callback and msg type, no extra ROS param declared."""

    def callback(msg: String) -> None:
        pass

    test_node.register_obk_subscription(key="test_sub", callback=callback, msg_type=String)
    assert any(s["key"] == "test_sub" and s["callback"] is callback for s in test_node._obk_sub_settings)


def test_register_obk_timer(test_node: ObeliskNode) -> None:
    """register_obk_timer stores the callback, no extra ROS param declared."""

    def callback() -> None:
        pass

    test_node.register_obk_timer(key="test_timer", callback=callback)
    assert any(s["key"] == "test_timer" and s["callback"] is callback for s in test_node._obk_timer_settings)


def test_create_publisher(test_node: ObeliskNode) -> None:
    """The underlying create_publisher still works for direct use cases."""
    pub = test_node.create_publisher(ocm.PositionSetpoint, "test_topic", 10)
    assert isinstance(pub, Publisher)


def test_create_subscription(test_node: ObeliskNode) -> None:
    """The underlying create_subscription still works for direct use cases."""

    def callback(msg: ocm.PositionSetpoint) -> None:
        pass

    sub = test_node.create_subscription(ocm.PositionSetpoint, "test_topic", callback, 10)
    assert isinstance(sub, Subscription)


def test_lifecycle_callbacks(test_node: ObeliskNode) -> None:
    """All lifecycle callbacks return SUCCESS for an empty ObeliskNode."""
    assert test_node.on_configure(None) == TransitionCallbackReturn.SUCCESS
    assert test_node.on_activate(None) == TransitionCallbackReturn.SUCCESS
    assert test_node.on_deactivate(None) == TransitionCallbackReturn.SUCCESS
    assert test_node.on_cleanup(None) == TransitionCallbackReturn.SUCCESS
    assert test_node.on_shutdown(None) == TransitionCallbackReturn.SUCCESS


def test_callback_group_creation(test_node: ObeliskNode, set_node_parameters: Callable) -> None:
    """Callback groups specified in obelisk_settings are instantiated and bound as attributes."""
    _set_obelisk_settings(
        test_node,
        {
            "callback_groups": {
                "group1": "MutuallyExclusiveCallbackGroup",
                "group2": "ReentrantCallbackGroup",
            }
        },
        set_node_parameters,
    )
    test_node.on_configure(None)
    assert isinstance(test_node.group1, MutuallyExclusiveCallbackGroup)
    assert isinstance(test_node.group2, ReentrantCallbackGroup)


def test_publisher_creation_from_settings(test_node: ObeliskNode, set_node_parameters: Callable) -> None:
    """A publisher entry in obelisk_settings.publishers becomes a real rclpy Publisher."""
    test_node.register_obk_publisher(key="test_pub", msg_type=ocm.PositionSetpoint)
    _set_obelisk_settings(
        test_node,
        {
            "publishers": [
                {"key": "test_pub", "topic": "/test_topic", "history_depth": 10},
            ]
        },
        set_node_parameters,
    )
    test_node.on_configure(None)

    assert "test_pub" in test_node.obk_publishers
    assert isinstance(test_node.obk_publishers["test_pub"], Publisher)


def test_subscription_creation_from_settings(test_node: ObeliskNode, set_node_parameters: Callable) -> None:
    """A subscriber entry in obelisk_settings.subscribers becomes a real rclpy Subscription."""

    def callback(msg: ocm.PositionSetpoint) -> None:
        pass

    test_node.register_obk_subscription(key="test_sub", callback=callback, msg_type=ocm.PositionSetpoint)
    _set_obelisk_settings(
        test_node,
        {
            "subscribers": [
                {"key": "test_sub", "topic": "/test_topic", "history_depth": 10},
            ]
        },
        set_node_parameters,
    )
    test_node.on_configure(None)

    assert "test_sub" in test_node.obk_subscriptions
    assert isinstance(test_node.obk_subscriptions["test_sub"], Subscription)


def test_timer_creation_from_settings(test_node: ObeliskNode, set_node_parameters: Callable) -> None:
    """A timer entry in obelisk_settings.timers becomes a real rclpy Timer."""

    def callback() -> None:
        pass

    test_node.register_obk_timer(key="test_timer", callback=callback)
    _set_obelisk_settings(
        test_node,
        {"timers": [{"key": "test_timer", "timer_period_sec": 1.0}]},
        set_node_parameters,
    )
    test_node.on_configure(None)

    assert "test_timer" in test_node.obk_timers
    assert isinstance(test_node.obk_timers["test_timer"], Timer)


def test_invalid_callback_group_type(test_node: ObeliskNode, set_node_parameters: Callable) -> None:
    """An unknown callback-group type raises ValueError at on_configure time."""
    _set_obelisk_settings(test_node, {"callback_groups": {"bad": "NotARealGroupType"}}, set_node_parameters)
    with pytest.raises(ValueError):
        test_node.on_configure(None)
