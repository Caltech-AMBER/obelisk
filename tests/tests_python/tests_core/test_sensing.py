"""Tests for ObeliskSensor (post-config-string-refactor)."""

from typing import Any, Callable, Generator

import obelisk_sensor_msgs.msg as osm
import pytest
import yaml
from rclpy.lifecycle import TransitionCallbackReturn

from obelisk_py.core.sensing import ObeliskSensor


@pytest.fixture
def test_sensor(ros_context: Any) -> Generator[ObeliskSensor, None, None]:
    """Fixture for the ObeliskSensor class."""
    sensor = ObeliskSensor("test_sensor")
    yield sensor
    sensor.destroy_node()


def test_sensor_initialization(test_sensor: ObeliskSensor) -> None:
    """The sensor is named correctly and starts without a sensor publisher."""
    assert test_sensor.get_name() == "test_sensor"
    assert not test_sensor._has_sensor_publisher


def test_sensor_configuration_without_publisher(test_sensor: ObeliskSensor) -> None:
    """ObeliskSensor without any sensor publisher fails the assert in on_configure."""
    with pytest.raises(AssertionError):
        test_sensor.on_configure(None)


def test_sensor_configuration_with_publisher(test_sensor: ObeliskSensor, set_node_parameters: Callable) -> None:
    """ObeliskSensor with a registered sensor publisher and a matching settings entry configures cleanly."""
    test_sensor.register_obk_publisher(key="test_pub", msg_type=osm.ObkJointEncoders)
    settings = {"publishers": [{"key": "test_pub", "topic": "/test_topic", "history_depth": 10}]}
    set_node_parameters(test_sensor, {"obelisk_settings": yaml.safe_dump(settings)})

    result = test_sensor.on_configure(None)
    assert result == TransitionCallbackReturn.SUCCESS
    assert test_sensor._has_sensor_publisher


def test_sensor_lifecycle_callbacks(test_sensor: ObeliskSensor) -> None:
    """Lifecycle callbacks return SUCCESS for a bare sensor (no on_configure called)."""
    assert test_sensor.on_activate(None) == TransitionCallbackReturn.SUCCESS
    assert test_sensor.on_deactivate(None) == TransitionCallbackReturn.SUCCESS
    assert test_sensor.on_cleanup(None) == TransitionCallbackReturn.SUCCESS
    assert test_sensor.on_shutdown(None) == TransitionCallbackReturn.SUCCESS
