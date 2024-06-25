from typing import Any, Callable, Dict, Generator, List

import obelisk_control_msgs.msg as ocm
import obelisk_estimator_msgs.msg as oem
import pytest
import rclpy

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
