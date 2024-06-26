from typing import Any, Callable, Dict, Generator, List

import obelisk_sensor_msgs.msg as osm
import pytest
import rclpy
from rclpy.exceptions import ParameterUninitializedException
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.publisher import Publisher

from obelisk_py.obelisk_typing import ObeliskMsg, ObeliskSensorMsg, is_in_bound
from obelisk_py.sensing import ObeliskSensor

# ##### #
# SETUP #
# ##### #


class TestObeliskSensor(ObeliskSensor):
    """Test ObeliskSensor class."""

    def publish_measurement1(self) -> ObeliskSensorMsg:
        """Publish measurement 1."""
        return osm.JointEncoders()

    def publish_measurement2(self) -> ObeliskSensorMsg:
        """Publish measurement 2."""
        return osm.JointEncoders()


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
                "msg_type:JointEncoders,"
                "topic:/obelisk/test_sensor/sensor1,"
                "history_depth:10,"
                "callback_group:None,"
                "non_obelisk:False"
            ),
            (
                "msg_type:JointEncoders,"
                "topic:/obelisk/test_sensor/sensor2,"
                "history_depth:10,"
                "callback_group:None,"
                "non_obelisk:False"
            ),
        ],
    }
    set_node_parameters(test_sensor, parameter_dict)
    test_sensor.on_configure(None)
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
    configured_sensor.on_cleanup(None)
    check_node_attributes(configured_sensor, sensing_parameter_names + component_names, should_exist=False)


def test_sensor_functionality(configured_sensor: TestObeliskSensor) -> None:
    """Test sensor functionality."""
    obk_sensor_msg1 = configured_sensor.publish_measurement1()
    obk_sensor_msg2 = configured_sensor.publish_measurement2()

    for msg in [obk_sensor_msg1, obk_sensor_msg2]:
        assert isinstance(msg, osm.JointEncoders)
        assert is_in_bound(type(msg), ObeliskSensorMsg)
        assert is_in_bound(type(msg), ObeliskMsg)


def test_on_configure_success(configured_sensor: TestObeliskSensor) -> None:
    """Test successful configuration of the sensor."""
    result = configured_sensor.on_configure(None)
    assert result == TransitionCallbackReturn.SUCCESS
    assert len(configured_sensor.publisher_sensors) == 2  # noqa: PLR2004
    assert all(isinstance(pub, Publisher) for pub in configured_sensor.publisher_sensors)


def test_on_configure_empty_config_strings(test_sensor: TestObeliskSensor) -> None:
    """Test configuration with empty pub_sensor_config_strs."""
    test_sensor.set_parameters([rclpy.Parameter("pub_sensor_config_strs", value=[""])])
    with pytest.raises(AssertionError, match="pub_sensor_config_strs must be a non-empty list of strings."):
        test_sensor.on_configure(None)


def test_on_configure_missing_parameter(test_sensor: TestObeliskSensor) -> None:
    """Test configuration with missing pub_sensor_config_strs parameter."""
    with pytest.raises(ParameterUninitializedException):
        test_sensor.on_configure(None)


def test_on_cleanup(configured_sensor: TestObeliskSensor) -> None:
    """Test cleanup of the sensor."""
    result = configured_sensor.on_cleanup(None)
    assert result == TransitionCallbackReturn.SUCCESS
    assert not hasattr(configured_sensor, "publisher_sensors")
    assert not hasattr(configured_sensor, "pub_sensor_config_strs")


def test_publisher_creation(configured_sensor: TestObeliskSensor) -> None:
    """Test creation of publishers based on configuration strings."""
    assert len(configured_sensor.publisher_sensors) == 2  # noqa: PLR2004
    assert all(isinstance(pub, Publisher) for pub in configured_sensor.publisher_sensors)


@pytest.mark.parametrize(
    "invalid_config",
    [
        ["invalid_config_string"],
        ["msg_type:InvalidType,topic:/test/topic"],
        ["msg_type:JointEncoders,topic:/test/topic,invalid_key:value"],
    ],
)
def test_invalid_publisher_config(test_sensor: TestObeliskSensor, invalid_config: List[str]) -> None:
    """Test handling of invalid publisher configuration strings."""
    test_sensor.set_parameters([rclpy.Parameter("pub_sensor_config_strs", value=invalid_config)])
    with pytest.raises(Exception) as exc_info:  # The exact exception type may vary based on implementation
        test_sensor.on_configure(None)
    assert isinstance(exc_info.value, Exception)


def test_publish_measurement_type(configured_sensor: TestObeliskSensor) -> None:
    """Test that published measurements are of the correct type."""
    measurement1 = configured_sensor.publish_measurement1()
    measurement2 = configured_sensor.publish_measurement2()
    assert isinstance(measurement1, osm.JointEncoders)
    assert isinstance(measurement2, osm.JointEncoders)


@pytest.mark.parametrize("method_name", ["publish_measurement1", "publish_measurement2"])
def test_publish_measurement_bounds(configured_sensor: TestObeliskSensor, method_name: str) -> None:
    """Test that published measurements are within the expected bounds."""
    method = getattr(configured_sensor, method_name)
    measurement = method()
    assert is_in_bound(type(measurement), ObeliskSensorMsg)
    assert is_in_bound(type(measurement), ObeliskMsg)


def test_multiple_sensors(test_sensor: TestObeliskSensor, set_node_parameters: Callable[[Any, Dict], None]) -> None:
    """Test configuration with multiple sensors."""
    parameter_dict = {
        "pub_sensor_config_strs": [
            "msg_type:JointEncoders,topic:/sensor1,history_depth:10,callback_group:None,non_obelisk:False",
            "msg_type:JointEncoders,topic:/sensor2,history_depth:5,callback_group:None,non_obelisk:False",
            "msg_type:JointEncoders,topic:/sensor3,history_depth:1,callback_group:None,non_obelisk:False",
        ],
    }
    set_node_parameters(test_sensor, parameter_dict)
    test_sensor.on_configure(None)
    assert len(test_sensor.publisher_sensors) == len(parameter_dict["pub_sensor_config_strs"])


def test_lifecycle_transitions(configured_sensor: TestObeliskSensor) -> None:
    """Test full lifecycle transitions of the sensor."""
    assert configured_sensor.on_configure(None) == TransitionCallbackReturn.SUCCESS
    assert configured_sensor.on_cleanup(None) == TransitionCallbackReturn.SUCCESS

    assert configured_sensor.on_configure(None) == TransitionCallbackReturn.SUCCESS
    assert configured_sensor.on_shutdown(None) == TransitionCallbackReturn.SUCCESS


def test_inheritance(configured_sensor: TestObeliskSensor) -> None:
    """Test that ObeliskSensor correctly inherits from ObeliskNode."""
    from obelisk_py.node import ObeliskNode

    assert isinstance(configured_sensor, ObeliskNode)


def test_publisher_topics(configured_sensor: TestObeliskSensor) -> None:
    """Test that publishers are created with the correct topics."""
    expected_topics = ["/obelisk/test_sensor/sensor1", "/obelisk/test_sensor/sensor2"]
    actual_topics = [pub.topic_name for pub in configured_sensor.publisher_sensors]
    assert actual_topics == expected_topics


def test_publisher_qos(configured_sensor: TestObeliskSensor) -> None:
    """Test that publishers are created with the correct QoS settings."""
    for pub in configured_sensor.publisher_sensors:
        assert pub.qos_profile.history == rclpy.qos.QoSHistoryPolicy.KEEP_LAST
        assert pub.qos_profile.depth == 10  # noqa: PLR2004


def test_callback_group_creation(configured_sensor: TestObeliskSensor) -> None:
    """Test that callback groups are created correctly."""
    assert hasattr(configured_sensor, "test_cbg")
    from rclpy.callback_groups import ReentrantCallbackGroup

    assert isinstance(configured_sensor.test_cbg, ReentrantCallbackGroup)


def test_parameter_types(configured_sensor: TestObeliskSensor) -> None:
    """Test that parameters are of the correct types."""
    pub_sensor_config_strs = configured_sensor.get_parameter("pub_sensor_config_strs").value
    assert isinstance(pub_sensor_config_strs, list)
    assert all(isinstance(config_str, str) for config_str in pub_sensor_config_strs)
