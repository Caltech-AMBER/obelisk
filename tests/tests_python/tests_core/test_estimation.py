"""Tests for ObeliskEstimator (post-config-string-refactor)."""

from typing import Any, Callable, Generator, Type

import pytest
import yaml
from obelisk_sensor_msgs.msg import ObkJointEncoders
from rclpy.lifecycle.node import TransitionCallbackReturn
from rclpy.publisher import Publisher
from rclpy.timer import Timer
from std_msgs.msg import String

from obelisk_py.core.estimation import ObeliskEstimator
from obelisk_py.core.obelisk_typing import ObeliskEstimatorMsg, ObeliskSensorMsg


class TestEstimator(ObeliskEstimator):
    """A concrete implementation of ObeliskEstimator for testing purposes."""

    def __init__(self, node_name: str, est_msg_type: Type) -> None:
        """Initialize the TestEstimator."""
        super().__init__(node_name, est_msg_type)
        self.register_obk_subscription(
            key="sub_sensor",
            callback=self.update_sensor,
            msg_type=ObkJointEncoders,
        )

    def update_sensor(self, sensor_msg: ObeliskSensorMsg) -> None:
        """Update the sensor data."""
        pass

    def compute_state_estimate(self) -> ObeliskEstimatorMsg:
        """Compute the state estimate."""
        return ObeliskEstimatorMsg()


@pytest.fixture
def test_estimator(ros_context: Any) -> Generator[TestEstimator, None, None]:
    """Fixture for the TestEstimator class."""
    estimator = TestEstimator("test_estimator", String)
    yield estimator
    estimator.destroy_node()


def test_estimator_initialization(test_estimator: TestEstimator) -> None:
    """The estimator is named correctly and declares the obelisk_settings parameter."""
    assert test_estimator.get_name() == "test_estimator"
    assert test_estimator.has_parameter("obelisk_settings")


def test_timer_registration(test_estimator: TestEstimator) -> None:
    """The compute_state_estimate timer is registered under key 'timer_est'."""
    timer_setting = next(s for s in test_estimator._obk_timer_settings if s["key"] == "timer_est")
    assert timer_setting["callback"] == test_estimator.compute_state_estimate


def test_subscription_registration(test_estimator: TestEstimator) -> None:
    """The sensor subscription is registered under key 'sub_sensor' with the right callback."""
    sub_setting = next(s for s in test_estimator._obk_sub_settings if s["key"] == "sub_sensor")
    assert sub_setting["callback"] == test_estimator.update_sensor


def test_estimator_configuration(test_estimator: TestEstimator, set_node_parameters: Callable) -> None:
    """The estimator wires up timer/publisher/subscriber from obelisk_settings."""
    settings = {
        "timers": [{"key": "timer_est", "timer_period_sec": 0.1}],
        "publishers": [{"key": "pub_est", "topic": "/test_estimate", "history_depth": 10}],
        "subscribers": [{"key": "sub_sensor", "topic": "/test_sensor", "history_depth": 10}],
    }
    set_node_parameters(test_estimator, {"obelisk_settings": yaml.safe_dump(settings)})
    result = test_estimator.on_configure(None)

    assert result == TransitionCallbackReturn.SUCCESS
    assert "timer_est" in test_estimator.obk_timers
    assert isinstance(test_estimator.obk_timers["timer_est"], Timer)
    assert "pub_est" in test_estimator.obk_publishers
    assert isinstance(test_estimator.obk_publishers["pub_est"], Publisher)
    assert "sub_sensor" in test_estimator.obk_subscriptions


def test_abstract_methods() -> None:
    """ObeliskEstimator's abstract methods must be implemented by subclasses."""
    with pytest.raises(TypeError):

        class IncompleteEstimator(ObeliskEstimator):
            pass

        IncompleteEstimator("incomplete_estimator")

    class CompleteEstimator(ObeliskEstimator):
        def compute_state_estimate(self) -> ObeliskEstimatorMsg:
            return ObeliskEstimatorMsg()

    complete_estimator = CompleteEstimator("complete_estimator", String)
    assert hasattr(complete_estimator, "compute_state_estimate")
