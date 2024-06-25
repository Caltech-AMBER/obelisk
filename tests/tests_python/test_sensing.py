from typing import Any, Callable, Dict, Generator, List

import obelisk_sensor_msgs.msg as osm
import pytest
import rclpy

from obelisk_py.obelisk_typing import ObeliskMsg, ObeliskSensorMsg, is_in_bound
from obelisk_py.sensing import ObeliskSensor

# ##### #
# SETUP #
# ##### #


class TestObeliskSensor(ObeliskSensor):
    """Test ObeliskSensor class."""

    def publish_measurement1(self) -> ObeliskSensorMsg:
        """Publish measurement 1."""
        return osm.JointEncoder()

    def publish_measurement2(self) -> ObeliskSensorMsg:
        """Publish measurement 2."""
        return osm.JointEncoder()


@pytest.fixture
def test_sensor(ros_context: Any) -> Generator[TestObeliskSensor, None, None]:
    """Fixture for the TestObeliskSensor class."""
    sensor = TestObeliskSensor("test_sensor")
    yield sensor
    sensor.destroy_node()


@pytest.fixture
def configured_sensor(
    test_sensor: TestObeliskSensor, set_node_parameters: Callable[[Any, Dict], None]
) -> TestObeliskSensor:
    """Fixture for the TestObeliskSensor class with parameters set."""
    parameter_dict = {
        "callback_group_config_strs": ["test_cbg:ReentrantCallbackGroup"],
        "pub_sensor_config_strs": [
            (
                "msg_type:JointEncoder,"
                "topic:/obelisk/test_sensor/sensor1,"
                "history_depth:10,"
                "callback_group:None,"
                "non_obelisk:False"
            ),
            (
                "msg_type:JointEncoder,"
                "topic:/obelisk/test_sensor/sensor2,"
                "history_depth:10,"
                "callback_group:None,"
                "non_obelisk:False"
            ),
        ],
    }
    set_node_parameters(test_sensor, parameter_dict)
    test_sensor.on_configure(test_sensor._state_machine.current_state)
    return test_sensor


@pytest.fixture
def sensing_parameter_names() -> List[str]:
    """Return the parameter names for the sensor."""
    return ["pub_sensor_config_strs"]


# ##### #
# TESTS #
# ##### #


def test_parameter_initialization(test_sensor: TestObeliskSensor, sensing_parameter_names: List[str]) -> None:
    """Test parameter initialization."""
    with pytest.raises(rclpy.exceptions.ParameterUninitializedException):
        test_sensor.get_parameters(sensing_parameter_names)


def test_parameter_setting(configured_sensor: TestObeliskSensor, sensing_parameter_names: List[str]) -> None:
    """Test parameter setting."""
    try:
        configured_sensor.get_parameters(sensing_parameter_names)
    except rclpy.exceptions.ParameterUninitializedException as e:
        pytest.fail(f"Failed to access parameters after setting them: {e}")


def test_configuration(
    configured_sensor: TestObeliskSensor,
    check_node_attributes: Callable[[Any, List[str], bool], None],
    sensing_parameter_names: List[str],
) -> None:
    """Test node configuration."""
    component_names = ["test_cbg", "publisher_sensors"]
    check_node_attributes(configured_sensor, sensing_parameter_names + component_names, should_exist=True)


def test_cleanup(
    configured_sensor: TestObeliskSensor,
    check_node_attributes: Callable[[Any, List[str], bool], None],
    sensing_parameter_names: List[str],
) -> None:
    """Test node cleanup."""
    component_names = ["test_cbg", "publisher_sensors"]
    configured_sensor.on_cleanup(configured_sensor._state_machine.current_state)
    check_node_attributes(configured_sensor, sensing_parameter_names + component_names, should_exist=False)


def test_sensor_functionality(configured_sensor: TestObeliskSensor) -> None:
    """Test sensor functionality."""
    obk_sensor_msg1 = configured_sensor.publish_measurement1()
    obk_sensor_msg2 = configured_sensor.publish_measurement2()

    for msg in [obk_sensor_msg1, obk_sensor_msg2]:
        assert isinstance(msg, osm.JointEncoder)
        assert is_in_bound(type(msg), ObeliskSensorMsg)
        assert is_in_bound(type(msg), ObeliskMsg)
