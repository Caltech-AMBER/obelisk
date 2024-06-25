from typing import Any, Callable, Dict, Generator, List

import obelisk_estimator_msgs.msg as oem
import obelisk_sensor_msgs.msg as osm
import pytest
import rclpy

from obelisk_py.estimation import ObeliskEstimator
from obelisk_py.obelisk_typing import ObeliskEstimatorMsg, ObeliskMsg, is_in_bound

# ##### #
# SETUP #
# ##### #


class TestObeliskEstimator(ObeliskEstimator):
    """Test ObeliskEstimator class."""

    def sensor_callback1(self, msg: ObeliskMsg) -> None:
        """Sensor callback 1."""
        pass

    def sensor_callback2(self, msg: ObeliskMsg) -> None:
        """Sensor callback 2."""
        pass

    def compute_state_estimate(self) -> ObeliskEstimatorMsg:
        """Compute the state estimate."""
        obk_est_msg = oem.EstimatedState()  # [NOTE] dummy implementation
        return obk_est_msg


@pytest.fixture
def test_estimator(ros_context: Any) -> Generator[TestObeliskEstimator, None, None]:
    """Fixture for the TestObeliskEstimator class."""
    estimator = TestObeliskEstimator("test_estimator")
    yield estimator
    estimator.destroy_node()


@pytest.fixture
def configured_estimator(
    test_estimator: TestObeliskEstimator, set_node_parameters: Callable[[Any, Dict], None]
) -> TestObeliskEstimator:
    """Fixture for the TestObeliskEstimator class with parameters set."""
    parameter_dict = {
        "callback_group_config_strs": ["test_cbg:ReentrantCallbackGroup"],
        "timer_est_config_str": "timer_period_sec:0.001,callback:compute_state_estimate,callback_group:None",
        "pub_est_config_str": (
            "msg_type:EstimatedState,"
            "topic:/obelisk/test_estimator/state_estimate,"
            "history_depth:10,"
            "callback_group:None,"
            "non_obelisk:False"
        ),
        "sub_sensor_config_strs": [
            (
                "msg_type:JointEncoder,"
                "topic:/obelisk/test_estimator/sensor1,"
                "callback:sensor_callback1,"
                "history_depth:10,"
                "callback_group:None,"
                "non_obelisk:False"
            ),
            (
                "msg_type:JointEncoder,"
                "topic:/obelisk/test_estimator/sensor2,"
                "callback:sensor_callback2,"
                "history_depth:10,"
                "callback_group:None,"
                "non_obelisk:False"
            ),
        ],
    }
    set_node_parameters(test_estimator, parameter_dict)
    test_estimator.on_configure(test_estimator._state_machine.current_state)
    return test_estimator


@pytest.fixture
def parameter_names() -> List[str]:
    """Parameter names for the ObeliskEstimator class."""
    return [
        "callback_group_config_strs",
        "timer_est_config_str",
        "pub_est_config_str",
        "sub_sensor_config_strs",
    ]


# ##### #
# TESTS #
# ##### #


def test_parameter_initialization(test_estimator: TestObeliskEstimator, parameter_names: List[str]) -> None:
    """Test parameter initialization."""
    with pytest.raises(rclpy.exceptions.ParameterUninitializedException):
        test_estimator.get_parameters(parameter_names)


def test_parameter_setting(configured_estimator: TestObeliskEstimator, parameter_names: List[str]) -> None:
    """Test parameter setting."""
    try:
        configured_estimator.get_parameters(parameter_names)
    except rclpy.exceptions.ParameterUninitializedException as e:
        pytest.fail(f"Failed to access parameters after setting them: {e}")


def test_configuration(
    configured_estimator: TestObeliskEstimator,
    check_node_attributes: Callable[[Any, List[str], bool], None],
    parameter_names: List[str],
) -> None:
    """Test node configuration."""
    component_names = ["test_cbg", "publisher_est", "timer_est", "subscriber_sensors"]
    check_node_attributes(configured_estimator, parameter_names + component_names, should_exist=True)


def test_cleanup(
    configured_estimator: TestObeliskEstimator,
    check_node_attributes: Callable[[Any, List[str], bool], None],
    parameter_names: List[str],
) -> None:
    """Test node cleanup."""
    component_names = ["test_cbg", "publisher_est", "timer_est", "subscriber_sensors"]
    configured_estimator.on_cleanup(configured_estimator._state_machine.current_state)
    check_node_attributes(configured_estimator, parameter_names + component_names, should_exist=False)


def test_estimator_functionality(configured_estimator: TestObeliskEstimator) -> None:
    """Test estimator functionality."""
    obk_estimator_msg = configured_estimator.compute_state_estimate()
    assert isinstance(obk_estimator_msg, oem.EstimatedState)
    assert is_in_bound(type(obk_estimator_msg), ObeliskEstimatorMsg)
    assert is_in_bound(type(obk_estimator_msg), ObeliskMsg)

    obk_sensor1_msg = osm.JointEncoder()  # [NOTE] dummy message
    try:
        configured_estimator.sensor_callback1(obk_sensor1_msg)
    except AttributeError:
        pytest.fail("Failed to call sensor_callback1")

    obk_sensor2_msg = osm.JointEncoder()  # [NOTE] dummy message
    try:
        configured_estimator.sensor_callback2(obk_sensor2_msg)
    except AttributeError:
        pytest.fail("Failed to call sensor_callback2")
