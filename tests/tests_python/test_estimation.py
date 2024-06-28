from typing import Any, Callable, Dict, Generator, List

import obelisk_estimator_msgs.msg as oem
import obelisk_sensor_msgs.msg as osm
import pytest
import rclpy
from rclpy.exceptions import ParameterUninitializedException
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.lifecycle.node import LifecycleState
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.timer import Timer

from obelisk_py.estimation import ObeliskEstimator
from obelisk_py.obelisk_typing import ObeliskEstimatorMsg, ObeliskMsg, is_in_bound

# ##### #
# SETUP #
# ##### #


class TestObeliskEstimator(ObeliskEstimator):
    """Test ObeliskEstimator class."""

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the estimator."""
        super().on_configure(state)
        sensor_callbacks = [self.sensor_callback1, self.sensor_callback2]
        for sensor_setting, sensor_callback in zip(self.sub_sensor_settings, sensor_callbacks):
            sub_sensor = self._create_subscription_from_config_str(sensor_setting, sensor_callback)
            self.subscriber_sensors.append(sub_sensor)

        return TransitionCallbackReturn.SUCCESS

    def sensor_callback1(self, msg: ObeliskMsg) -> None:
        """Sensor callback 1."""
        self.sensor1_data = 1

    def sensor_callback2(self, msg: ObeliskMsg) -> None:
        """Sensor callback 2."""
        self.sensor2_data = 2

    def compute_state_estimate(self) -> ObeliskEstimatorMsg:
        """Compute the state estimate."""
        return oem.EstimatedState()


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
        "callback_group_settings": "test_cbg:ReentrantCallbackGroup",
        "timer_est_setting": "timer_period_sec:0.001,callback_group:None",
        "pub_est_setting": (
            "msg_type:EstimatedState,"
            "topic:/obelisk/test_estimator/state_estimate,"
            "history_depth:10,"
            "callback_group:None,"
            "non_obelisk:False"
        ),
        "sub_sensor_settings": [
            (
                "msg_type:JointEncoders,"
                "topic:/obelisk/test_estimator/sensor1,"
                "history_depth:10,"
                "callback_group:None,"
                "non_obelisk:False"
            ),
            (
                "msg_type:JointEncoders,"
                "topic:/obelisk/test_estimator/sensor2,"
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
    """Return the parameter names for the estimator."""
    return [
        "callback_group_settings",
        "timer_est_setting",
        "pub_est_setting",
        "sub_sensor_settings",
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
    """Test the configuration of the estimator."""
    component_names = ["test_cbg", "publisher_est", "timer_est", "subscriber_sensors"]
    check_node_attributes(configured_estimator, parameter_names + component_names, should_exist=True)


def test_cleanup(
    configured_estimator: TestObeliskEstimator,
    check_node_attributes: Callable[[Any, List[str], bool], None],
    parameter_names: List[str],
) -> None:
    """Test the cleanup of the estimator."""
    component_names = ["test_cbg", "publisher_est", "timer_est", "subscriber_sensors"]
    configured_estimator.on_cleanup(configured_estimator._state_machine.current_state)
    check_node_attributes(configured_estimator, parameter_names + component_names, should_exist=False)


def test_estimator_functionality(configured_estimator: TestObeliskEstimator) -> None:
    """Test the functionality of the estimator."""
    joint_encoder_msg1 = osm.JointEncoders()
    joint_encoder_msg2 = osm.JointEncoders()

    configured_estimator.sensor_callback1(joint_encoder_msg1)
    configured_estimator.sensor_callback2(joint_encoder_msg2)

    assert configured_estimator.sensor1_data == 1
    assert configured_estimator.sensor2_data == 2  # noqa: PLR2004

    obk_est_msg = configured_estimator.compute_state_estimate()
    assert isinstance(obk_est_msg, oem.EstimatedState)
    assert is_in_bound(type(obk_est_msg), ObeliskEstimatorMsg)
    assert is_in_bound(type(obk_est_msg), ObeliskMsg)


def test_on_configure_success(configured_estimator: TestObeliskEstimator) -> None:
    """Test successful configuration of the estimator."""
    result = configured_estimator.on_configure(None)
    assert result == TransitionCallbackReturn.SUCCESS
    assert hasattr(configured_estimator, "timer_est")
    assert isinstance(configured_estimator.timer_est, Timer)
    assert hasattr(configured_estimator, "publisher_est")
    assert isinstance(configured_estimator.publisher_est, Publisher)
    assert hasattr(configured_estimator, "subscriber_sensors")
    assert isinstance(configured_estimator.subscriber_sensors, list)
    assert all(isinstance(sub, Subscription) for sub in configured_estimator.subscriber_sensors)


def test_on_configure_missing_parameters(test_estimator: TestObeliskEstimator) -> None:
    """Test configuration with missing parameters."""
    with pytest.raises(ParameterUninitializedException):
        test_estimator.on_configure(None)


def test_on_cleanup(configured_estimator: TestObeliskEstimator) -> None:
    """Test cleanup of the estimator."""
    result = configured_estimator.on_cleanup(None)
    assert result == TransitionCallbackReturn.SUCCESS
    assert not hasattr(configured_estimator, "timer_est")
    assert not hasattr(configured_estimator, "publisher_est")
    assert not hasattr(configured_estimator, "subscriber_sensors")
    assert not hasattr(configured_estimator, "timer_est_setting")
    assert not hasattr(configured_estimator, "pub_est_setting")
    assert not hasattr(configured_estimator, "sub_sensor_settings")


def test_on_activate(configured_estimator: TestObeliskEstimator) -> None:
    """Test activation of the estimator."""
    result = configured_estimator.on_activate(None)
    assert result == TransitionCallbackReturn.SUCCESS
    # Check if timer is reset (this might need to be mocked in a real implementation)


def test_on_deactivate(configured_estimator: TestObeliskEstimator) -> None:
    """Test deactivation of the estimator."""
    configured_estimator.on_activate(None)
    result = configured_estimator.on_deactivate(None)
    assert result == TransitionCallbackReturn.SUCCESS
    # Check if timer is cancelled (this might need to be mocked in a real implementation)


def test_sensor_callbacks(configured_estimator: TestObeliskEstimator) -> None:
    """Test the sensor callback methods."""
    joint_encoder_msg1 = osm.JointEncoders()
    joint_encoder_msg2 = osm.JointEncoders()

    configured_estimator.sensor_callback1(joint_encoder_msg1)
    configured_estimator.sensor_callback2(joint_encoder_msg2)

    assert configured_estimator.sensor1_data == 1
    assert configured_estimator.sensor2_data == 2  # noqa: PLR2004


def test_compute_state_estimate(configured_estimator: TestObeliskEstimator) -> None:
    """Test the compute_state_estimate method."""
    estimate_msg = configured_estimator.compute_state_estimate()
    assert isinstance(estimate_msg, oem.EstimatedState)
    assert is_in_bound(type(estimate_msg), ObeliskEstimatorMsg)
    assert is_in_bound(type(estimate_msg), ObeliskMsg)


def test_timer_callback_assignment(configured_estimator: TestObeliskEstimator) -> None:
    """Test that the compute_state_estimate method is set as the timer callback."""
    assert configured_estimator.timer_est.callback == configured_estimator.compute_state_estimate


def test_subscriber_callback_assignments(configured_estimator: TestObeliskEstimator) -> None:
    """Test that the sensor callback methods are set as the subscriber callbacks."""
    assert configured_estimator.subscriber_sensors[0].callback == configured_estimator.sensor_callback1
    assert configured_estimator.subscriber_sensors[1].callback == configured_estimator.sensor_callback2


@pytest.mark.parametrize(
    "invalid_config",
    [
        {"timer_est_setting": "invalid_setting"},
        {"pub_est_setting": "invalid_setting"},
        {"sub_sensor_settings": ["invalid_setting"]},
    ],
)
def test_invalid_configuration(test_estimator: TestObeliskEstimator, invalid_config: Dict[str, Any]) -> None:
    """Test handling of invalid configuration strings."""
    for key, value in invalid_config.items():
        test_estimator.set_parameters([rclpy.Parameter(key, value=value)])
    with pytest.raises(Exception) as exc_info:  # The exact exception type may vary based on implementation
        test_estimator.on_configure(None)
    assert isinstance(exc_info.value, Exception)


def test_lifecycle_transitions(configured_estimator: TestObeliskEstimator) -> None:
    """Test full lifecycle transitions of the estimator."""
    assert configured_estimator.on_configure(None) == TransitionCallbackReturn.SUCCESS
    assert configured_estimator.on_activate(None) == TransitionCallbackReturn.SUCCESS
    assert configured_estimator.on_deactivate(None) == TransitionCallbackReturn.SUCCESS
    assert configured_estimator.on_cleanup(None) == TransitionCallbackReturn.SUCCESS

    assert configured_estimator.on_configure(None) == TransitionCallbackReturn.SUCCESS
    assert configured_estimator.on_shutdown(None) == TransitionCallbackReturn.SUCCESS


def test_inheritance(configured_estimator: TestObeliskEstimator) -> None:
    """Test that ObeliskEstimator correctly inherits from ObeliskNode."""
    from obelisk_py.node import ObeliskNode

    assert isinstance(configured_estimator, ObeliskNode)
