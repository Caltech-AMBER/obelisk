from typing import Any, Callable, Dict, Generator, List

import obelisk_control_msgs.msg as ocm
import obelisk_estimator_msgs.msg as oem
import pytest
import rclpy
from rclpy.exceptions import ParameterUninitializedException
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.timer import Timer

from obelisk_py.control import ObeliskController
from obelisk_py.obelisk_typing import ObeliskControlMsg, ObeliskEstimatorMsg, ObeliskMsg, is_in_bound

# ##### #
# SETUP #
# ##### #


class TestObeliskController(ObeliskController):
    """Test ObeliskController class."""

    def update_x_hat(self, x_hat_msg: ObeliskEstimatorMsg) -> None:
        """Update the state estimate."""
        self.x_hat = 1

    def compute_control(self) -> ObeliskControlMsg:
        """Compute the control."""
        return ocm.PositionSetpoint()


@pytest.fixture
def test_controller(ros_context: Any) -> Generator[TestObeliskController, None, None]:
    """Fixture for the TestObeliskController class."""
    controller = TestObeliskController("test_controller")
    yield controller
    controller.destroy_node()


@pytest.fixture
def configured_controller(
    test_controller: TestObeliskController, set_node_parameters: Callable[[Any, Dict], None]
) -> TestObeliskController:
    """Fixture for the TestObeliskController class with parameters set."""
    parameter_dict = {
        "callback_group_config_strs": ["test_cbg:ReentrantCallbackGroup"],
        "timer_ctrl_config_str": "timer_period_sec:0.001,callback:compute_control,callback_group:None",
        "pub_ctrl_config_str": (
            "msg_type:PositionSetpoint,"
            "topic:/obelisk/test_controller/control,"
            "history_depth:10,"
            "callback_group:None,"
            "non_obelisk:False"
        ),
        "sub_est_config_str": (
            "msg_type:EstimatedState,"
            "topic:/obelisk/test_controller/state_estimate,"
            "callback:update_x_hat,"
            "history_depth:10,"
            "callback_group:None,"
            "non_obelisk:False"
        ),
    }
    set_node_parameters(test_controller, parameter_dict)
    test_controller.on_configure(test_controller._state_machine.current_state)
    return test_controller


@pytest.fixture
def parameter_names() -> List[str]:
    """Return the parameter names for the controller."""
    return [
        "callback_group_config_strs",
        "timer_ctrl_config_str",
        "pub_ctrl_config_str",
        "sub_est_config_str",
    ]


# ##### #
# TESTS #
# ##### #


def test_parameter_initialization(test_controller: TestObeliskController, parameter_names: List[str]) -> None:
    """Test parameter initialization."""
    with pytest.raises(rclpy.exceptions.ParameterUninitializedException):
        test_controller.get_parameters(parameter_names)


def test_parameter_setting(configured_controller: TestObeliskController, parameter_names: List[str]) -> None:
    """Test parameter setting."""
    try:
        configured_controller.get_parameters(parameter_names)
    except rclpy.exceptions.ParameterUninitializedException as e:
        pytest.fail(f"Failed to access parameters after setting them: {e}")


def test_configuration(
    configured_controller: TestObeliskController,
    check_node_attributes: Callable[[Any, List[str], bool], None],
    parameter_names: List[str],
) -> None:
    """Test the configuration of the controller."""
    component_names = ["test_cbg", "publisher_ctrl", "timer_ctrl", "subscriber_est"]
    check_node_attributes(configured_controller, parameter_names + component_names, should_exist=True)


def test_cleanup(
    configured_controller: TestObeliskController,
    check_node_attributes: Callable[[Any, List[str], bool], None],
    parameter_names: List[str],
) -> None:
    """Test the cleanup of the controller."""
    component_names = ["test_cbg", "publisher_ctrl", "timer_ctrl", "subscriber_est"]
    configured_controller.on_cleanup(configured_controller._state_machine.current_state)
    check_node_attributes(configured_controller, parameter_names + component_names, should_exist=False)


def test_controller_functionality(configured_controller: TestObeliskController) -> None:
    """Test the functionality of the controller."""
    estimated_state_msg = oem.EstimatedState()
    configured_controller.update_x_hat(estimated_state_msg)
    assert configured_controller.x_hat == 1

    obk_ctrl_msg = configured_controller.compute_control()
    assert isinstance(obk_ctrl_msg, ocm.PositionSetpoint)
    assert is_in_bound(type(obk_ctrl_msg), ObeliskControlMsg)
    assert is_in_bound(type(obk_ctrl_msg), ObeliskMsg)


def test_on_configure_success(configured_controller: TestObeliskController) -> None:
    """Test successful configuration of the controller."""
    result = configured_controller.on_configure(None)
    assert result == TransitionCallbackReturn.SUCCESS
    assert hasattr(configured_controller, "timer_ctrl")
    assert isinstance(configured_controller.timer_ctrl, Timer)
    assert hasattr(configured_controller, "publisher_ctrl")
    assert isinstance(configured_controller.publisher_ctrl, Publisher)
    assert hasattr(configured_controller, "subscriber_est")
    assert isinstance(configured_controller.subscriber_est, Subscription)


def test_on_configure_missing_parameters(test_controller: TestObeliskController) -> None:
    """Test configuration with missing parameters."""
    with pytest.raises(ParameterUninitializedException):
        test_controller.on_configure(None)


def test_on_cleanup(configured_controller: TestObeliskController) -> None:
    """Test cleanup of the controller."""
    result = configured_controller.on_cleanup(None)
    assert result == TransitionCallbackReturn.SUCCESS
    assert not hasattr(configured_controller, "timer_ctrl")
    assert not hasattr(configured_controller, "publisher_ctrl")
    assert not hasattr(configured_controller, "subscriber_est")
    assert not hasattr(configured_controller, "timer_ctrl_config_str")
    assert not hasattr(configured_controller, "pub_ctrl_config_str")
    assert not hasattr(configured_controller, "sub_est_config_str")


def test_on_activate(configured_controller: TestObeliskController) -> None:
    """Test activation of the controller."""
    result = configured_controller.on_activate(None)
    assert result == TransitionCallbackReturn.SUCCESS
    assert configured_controller.x_hat is None


def test_on_deactivate(configured_controller: TestObeliskController) -> None:
    """Test deactivation of the controller."""
    configured_controller.on_activate(None)
    result = configured_controller.on_deactivate(None)
    assert result == TransitionCallbackReturn.SUCCESS
    # Check if timer is cancelled (this might need to be mocked in a real implementation)


def test_update_x_hat(configured_controller: TestObeliskController) -> None:
    """Test the update_x_hat method."""
    estimated_state_msg = oem.EstimatedState()
    configured_controller.update_x_hat(estimated_state_msg)
    assert configured_controller.x_hat == 1


def test_compute_control(configured_controller: TestObeliskController) -> None:
    """Test the compute_control method."""
    control_msg = configured_controller.compute_control()
    assert isinstance(control_msg, ocm.PositionSetpoint)
    assert is_in_bound(type(control_msg), ObeliskControlMsg)
    assert is_in_bound(type(control_msg), ObeliskMsg)


def test_timer_callback_assignment(configured_controller: TestObeliskController) -> None:
    """Test that the compute_control method is set as the timer callback."""
    assert configured_controller.timer_ctrl.callback == configured_controller.compute_control


def test_subscriber_callback_assignment(configured_controller: TestObeliskController) -> None:
    """Test that the update_x_hat method is set as the subscriber callback."""
    assert configured_controller.subscriber_est.callback == configured_controller.update_x_hat


@pytest.mark.parametrize(
    "invalid_config",
    [
        {"timer_ctrl_config_str": "invalid_config_string"},
        {"pub_ctrl_config_str": "invalid_config_string"},
        {"sub_est_config_str": "invalid_config_string"},
    ],
)
def test_invalid_configuration(test_controller: TestObeliskController, invalid_config: Dict[str, Any]) -> None:
    """Test handling of invalid configuration strings."""
    for key, value in invalid_config.items():
        test_controller.set_parameters([rclpy.Parameter(key, value=value)])
    with pytest.raises(Exception) as exc_info:  # The exact exception type may vary based on implementation
        test_controller.on_configure(None)
    assert isinstance(exc_info.value, Exception)


def test_lifecycle_transitions(configured_controller: TestObeliskController) -> None:
    """Test full lifecycle transitions of the controller."""
    assert configured_controller.on_configure(None) == TransitionCallbackReturn.SUCCESS
    assert configured_controller.on_activate(None) == TransitionCallbackReturn.SUCCESS
    assert configured_controller.on_deactivate(None) == TransitionCallbackReturn.SUCCESS
    assert configured_controller.on_cleanup(None) == TransitionCallbackReturn.SUCCESS

    assert configured_controller.on_configure(None) == TransitionCallbackReturn.SUCCESS
    assert configured_controller.on_shutdown(None) == TransitionCallbackReturn.SUCCESS


def test_inheritance(configured_controller: TestObeliskController) -> None:
    """Test that ObeliskController correctly inherits from ObeliskNode."""
    from obelisk_py.node import ObeliskNode

    assert isinstance(configured_controller, ObeliskNode)
