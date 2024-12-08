from typing import Any, Callable, Generator

import pytest
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.timer import Timer
from std_msgs.msg import String

from obelisk_py.core.control import ObeliskController
from obelisk_py.core.obelisk_typing import ObeliskControlMsg, ObeliskEstimatorMsg

# ##### #
# SETUP #
# ##### #


class TestController(ObeliskController):
    """A concrete implementation of ObeliskController for testing purposes."""

    def update_x_hat(self, x_hat_msg: ObeliskEstimatorMsg) -> None:
        """Update the state estimate."""
        pass

    def compute_control(self) -> ObeliskControlMsg:
        """Compute the control signal."""
        return ObeliskControlMsg()


@pytest.fixture
def test_controller(ros_context: Any) -> Generator[TestController, None, None]:
    """Fixture for the TestController class."""
    ctrl_msg_type = String
    est_msg_type = String
    controller = TestController("test_controller", ctrl_msg_type, est_msg_type)
    yield controller
    controller.destroy_node()


# ##### #
# TESTS #
# ##### #


def test_controller_initialization(test_controller: TestController) -> None:
    """Test the initialization of the ObeliskController.

    This test checks if the controller is properly initialized with the correct name and if the required timers,
    publishers, and subscriptions are registered.

    Parameters:
        test_controller: An instance of TestController.
    """
    assert test_controller.get_name() == "test_controller"
    assert test_controller.has_parameter("timer_ctrl_setting")
    assert test_controller.has_parameter("pub_ctrl_setting")
    assert test_controller.has_parameter("sub_est_setting")


def test_timer_registration(test_controller: TestController) -> None:
    """Test the registration of the control timer.

    This test verifies that the control timer is properly registered with the correct key and callback.

    Parameters:
        test_controller: An instance of TestController.
    """
    timer_setting = next(s for s in test_controller._obk_timer_settings if s["key"] == "timer_ctrl")
    assert timer_setting["callback"] == test_controller.compute_control


def test_subscription_registration(test_controller: TestController) -> None:
    """Test the registration of the estimator subscription.

    This test verifies that the estimator subscription is properly registered with the correct key, callback, and
    message type.

    Parameters:
        test_controller: An instance of TestController.
    """
    sub_setting = next(s for s in test_controller._obk_sub_settings if s["key"] == "sub_est")
    assert sub_setting["callback"] == test_controller.update_x_hat


def test_controller_configuration(test_controller: TestController, set_node_parameters: Callable) -> None:
    """Test the configuration of the ObeliskController.

    This test verifies that the controller can be properly configured with the necessary parameters and that the timers,
    publishers, and subscriptions are created correctly.

    Parameters:
        test_controller: An instance of TestController.
        set_node_parameters: A fixture to set node parameters.
    """
    set_node_parameters(
        test_controller,
        {
            "timer_ctrl_setting": "timer_period_sec:0.1",
            "pub_ctrl_setting": "topic:/test_control,msg_type:PositionSetpoint",
            "sub_est_setting": "topic:/test_estimate,msg_type:EstimatedState",
        },
    )
    test_controller.on_configure(None)

    assert "timer_ctrl" in test_controller.obk_timers
    assert isinstance(test_controller.obk_timers["timer_ctrl"], Timer)

    assert "pub_ctrl" in test_controller.obk_publishers
    assert isinstance(test_controller.obk_publishers["pub_ctrl"], Publisher)

    assert "sub_est" in test_controller.obk_subscriptions
    assert isinstance(test_controller.obk_subscriptions["sub_est"], Subscription)


def test_abstract_methods() -> None:
    """Test the abstract methods of ObeliskController.

    This test verifies that the abstract methods update_x_hat and compute_control are properly defined and must be
    implemented by subclasses.
    """
    with pytest.raises(TypeError):

        class IncompleteController(ObeliskController):
            pass

        IncompleteController("incomplete_controller")

    class CompleteController(ObeliskController):
        def update_x_hat(self, x_hat_msg: ObeliskEstimatorMsg) -> None:
            pass

        def compute_control(self) -> ObeliskControlMsg:
            return ObeliskControlMsg()

    complete_controller = CompleteController("complete_controller", String, String)
    assert hasattr(complete_controller, "update_x_hat")
    assert hasattr(complete_controller, "compute_control")
