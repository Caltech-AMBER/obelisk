from typing import Any, Callable, Generator

import obelisk_sensor_msgs.msg as osm
import pytest
from rclpy.lifecycle import TransitionCallbackReturn

from obelisk_py.core.sensing import ObeliskSensor


@pytest.fixture
def test_sensor(ros_context: Any) -> Generator[ObeliskSensor, None, None]:
    """Fixture for the ObeliskSensor class."""
    sensor = ObeliskSensor("test_sensor")
    yield sensor
    sensor.destroy_node()


def test_sensor_initialization(test_sensor: ObeliskSensor) -> None:
    """Test the initialization of the ObeliskSensor.

    This test checks if the sensor is properly initialized with the correct name.

    Parameters:
        test_sensor: An instance of ObeliskSensor.
    """
    assert test_sensor.get_name() == "test_sensor"
    assert not test_sensor._has_sensor_publisher


def test_sensor_configuration_without_publisher(test_sensor: ObeliskSensor) -> None:
    """Test the configuration of ObeliskSensor without a sensor publisher.

    This test verifies that configuring a sensor without a sensor publisher raises an assertion error.

    Parameters:
        test_sensor: An instance of ObeliskSensor.
    """
    with pytest.raises(AssertionError):
        test_sensor.on_configure(None)


def test_sensor_configuration_with_publisher(test_sensor: ObeliskSensor, set_node_parameters: Callable) -> None:
    """Test the configuration of ObeliskSensor with a sensor publisher.

    This test verifies that configuring a sensor with a valid sensor publisher succeeds.

    Parameters:
        test_sensor: An instance of ObeliskSensor.
        set_node_parameters: A fixture to set node parameters.
    """
    test_sensor.register_obk_publisher("test_pub_param", osm.ObkJointEncoders, key="test_pub")
    set_node_parameters(test_sensor, {"test_pub_param": "topic:/test_topic,msg_type:ObkJointEncoders,history_depth:10"})

    result = test_sensor.on_configure(None)
    assert result == TransitionCallbackReturn.SUCCESS
    assert test_sensor._has_sensor_publisher


def test_sensor_lifecycle_callbacks(test_sensor: ObeliskSensor) -> None:
    """Test the lifecycle callbacks of the ObeliskSensor.

    This test verifies that the lifecycle callbacks return the expected TransitionCallbackReturn.

    Parameters:
        test_sensor: An instance of ObeliskSensor.
    """
    assert test_sensor.on_activate(None) == TransitionCallbackReturn.SUCCESS
    assert test_sensor.on_deactivate(None) == TransitionCallbackReturn.SUCCESS
    assert test_sensor.on_cleanup(None) == TransitionCallbackReturn.SUCCESS
    assert test_sensor.on_shutdown(None) == TransitionCallbackReturn.SUCCESS
