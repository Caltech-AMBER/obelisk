from typing import Any, Callable, Generator

import pytest
import rclpy
from rclpy.lifecycle.node import TransitionCallbackReturn
from rclpy.publisher import Publisher
from rclpy.timer import Timer

from obelisk_py.core.estimation import ObeliskEstimator
from obelisk_py.core.obelisk_typing import ObeliskEstimatorMsg, ObeliskSensorMsg

# ##### #
# SETUP #
# ##### #


class TestEstimator(ObeliskEstimator):
    """A concrete implementation of ObeliskEstimator for testing purposes."""

    def __init__(self, node_name: str) -> None:
        """Initialize the TestEstimator."""
        super().__init__(node_name)
        self.register_obk_subscription(
            "sub_sensor_setting",
            self.update_sensor,
            key="sub_sensor",
            msg_type=None,  # generic, specified in config file
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
    estimator = TestEstimator("test_estimator")
    yield estimator
    estimator.destroy_node()


# ##### #
# TESTS #
# ##### #


def test_estimator_initialization(test_estimator: TestEstimator) -> None:
    """Test the initialization of the ObeliskEstimator.

    This test checks if the estimator is properly initialized with the correct name and if the required timers,
    publishers, and subscriptions are registered.

    Parameters:
        test_estimator: An instance of TestEstimator.
    """
    assert test_estimator.get_name() == "test_estimator"
    assert test_estimator.has_parameter("timer_est_setting")
    assert test_estimator.has_parameter("pub_est_setting")
    assert test_estimator.has_parameter("sub_sensor_setting")


def test_timer_registration(test_estimator: TestEstimator) -> None:
    """Test the registration of the estimation timer.

    This test verifies that the estimation timer is properly registered with the correct key and callback.

    Parameters:
        test_estimator: An instance of TestEstimator.
    """
    timer_setting = next(s for s in test_estimator._obk_timer_settings if s["key"] == "timer_est")
    assert timer_setting["callback"] == test_estimator.compute_state_estimate


def test_publisher_registration(test_estimator: TestEstimator) -> None:
    """Test the registration of the estimation publisher.

    This test verifies that the estimation publisher is properly registered with the correct key and message type.

    Parameters:
        test_estimator: An instance of TestEstimator.
    """
    pub_setting = next(s for s in test_estimator._obk_pub_settings if s["key"] == "pub_est")
    assert pub_setting["msg_type"] is None  # Should be specified in config file


def test_subscription_registration(test_estimator: TestEstimator) -> None:
    """Test the registration of the sensor subscription.

    This test verifies that the sensor subscription is properly registered with the correct key, callback, and
    message type.

    Parameters:
        test_estimator: An instance of TestEstimator.
    """
    sub_setting = next(s for s in test_estimator._obk_sub_settings if s["key"] == "sub_sensor")
    assert sub_setting["callback"] == test_estimator.update_sensor
    assert sub_setting["msg_type"] is None  # Should be specified in config file


def test_estimator_configuration(test_estimator: TestEstimator, set_node_parameters: Callable) -> None:
    """Test the configuration of the ObeliskEstimator.

    This test verifies that the estimator can be properly configured with the necessary parameters and that the
    timers, publishers, and subscriptions are created correctly.

    Parameters:
        test_estimator: An instance of TestEstimator.
        set_node_parameters: A fixture to set node parameters.
    """
    set_node_parameters(
        test_estimator,
        {
            "timer_est_setting": "timer_period_sec:0.1",
            "pub_est_setting": "topic:/test_estimate,msg_type:EstimatedState",
            "sub_sensor_setting": "topic:/test_sensor,msg_type:JointEncoders",
        },
    )
    result = test_estimator.on_configure(None)

    assert result == TransitionCallbackReturn.SUCCESS
    assert "timer_est" in test_estimator.obk_timers
    assert isinstance(test_estimator.obk_timers["timer_est"], Timer)
    assert "pub_est" in test_estimator.obk_publishers
    assert isinstance(test_estimator.obk_publishers["pub_est"], Publisher)
    assert "sub_sensor" in test_estimator.obk_subscriptions
    assert test_estimator._has_sensor_subscriber


def test_abstract_methods() -> None:
    """Test the abstract methods of ObeliskEstimator.

    This test verifies that the abstract method compute_state_estimate is properly defined and must be implemented
    by subclasses.
    """
    with pytest.raises(TypeError):

        class IncompleteEstimator(ObeliskEstimator):
            pass

        IncompleteEstimator("incomplete_estimator")

    class CompleteEstimator(ObeliskEstimator):
        def compute_state_estimate(self) -> ObeliskEstimatorMsg:
            return ObeliskEstimatorMsg()

    complete_estimator = CompleteEstimator("complete_estimator")
    assert hasattr(complete_estimator, "compute_state_estimate")


def test_missing_sensor_subscriber() -> None:
    """Test the requirement for at least one sensor subscriber.

    This test verifies that an assertion error is raised when no sensor subscriber is registered.
    """

    class NoSensorEstimator(ObeliskEstimator):
        def compute_state_estimate(self) -> ObeliskEstimatorMsg:
            return ObeliskEstimatorMsg()

    estimator = NoSensorEstimator("no_sensor_estimator")
    with pytest.raises(rclpy.exceptions.ParameterUninitializedException):
        estimator.on_configure(None)
