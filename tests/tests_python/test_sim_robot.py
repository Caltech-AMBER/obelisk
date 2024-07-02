from typing import Any, Callable, Dict, Generator, List

import obelisk_control_msgs.msg as ocm
import obelisk_sensor_msgs.msg as osm
import pytest
import rclpy
from rclpy.exceptions import ParameterUninitializedException
from rclpy.lifecycle import TransitionCallbackReturn

from obelisk_py.obelisk_typing import ObeliskControlMsg
from obelisk_py.robot import ObeliskRobot, ObeliskSimRobot

# ##### #
# SETUP #
# ##### #


class TestObeliskSimRobot(ObeliskSimRobot):
    """Test ObeliskSimRobot class."""

    def __init__(self, node_name: str) -> None:
        """Initialize the ObeliskSimRobot."""
        super().__init__(node_name)
        self.test_run_simulator_flag = False

    def apply_control(self, _: ObeliskControlMsg) -> None:
        """Apply control."""
        pass

    def publish_true_sim_state(self) -> osm.TrueSimState:
        """Publish the true simulation state."""
        return osm.TrueSimState()

    def run_simulator(self) -> None:
        """Run the simulator."""
        self.test_run_simulator_flag = True


@pytest.fixture
def test_sim_robot(ros_context: Any) -> Generator[TestObeliskSimRobot, None, None]:
    """Fixture for the TestObeliskSimRobot class."""
    sim_robot = TestObeliskSimRobot("test_sim_robot")
    yield sim_robot
    sim_robot.destroy_node()


@pytest.fixture
def configured_sim_robot(
    test_sim_robot: TestObeliskSimRobot, set_node_parameters: Callable[[Any, Dict], None]
) -> TestObeliskSimRobot:
    """Fixture for the TestObeliskSimRobot class with parameters set."""
    parameter_dict = {
        "callback_group_settings": "test_cbg:ReentrantCallbackGroup",
        "sub_ctrl_setting": (
            "msg_type:PositionSetpoint,"
            "topic:/obelisk/test_sim_robot/ctrl,"
            "history_depth:10,"
            "callback_group:None,"
            "non_obelisk:False"
        ),
        "timer_true_sim_state_setting": ("timer_period_sec:0.1,callback_group:None"),
        "pub_true_sim_state_setting": (
            "msg_type:TrueSimState,"
            "topic:/obelisk/test_sim_robot/true_sim_state,"
            "history_depth:10,"
            "callback_group:None,"
            "non_obelisk:False"
        ),
        # "mujoco_setting": {
        #     "model_path": "dummy/dummy.xml",
        #     "n_u": 1,
        #     "time_step": 0.002,
        #     "num_steps_per_viz": 5,
        #     "sensor_settings": (
        #         "[{topic=/obelisk/dummy/sensor|dt=0.001|sensor_type=jointpos|sensor_names=sensor_joint1}"
        #     )
        # }
    }
    set_node_parameters(test_sim_robot, parameter_dict)
    test_sim_robot.on_configure(test_sim_robot._state_machine.current_state)
    return test_sim_robot


@pytest.fixture
def sim_robot_parameter_names() -> List[str]:
    """Parameter names for the ObeliskSimRobot class."""
    return [
        "timer_true_sim_state_setting",
        "pub_true_sim_state_setting",
    ]


# ##### #
# TESTS #
# ##### #


def test_sim_robot_parameter_initialization(
    test_sim_robot: TestObeliskSimRobot, sim_robot_parameter_names: List[str]
) -> None:
    """Test parameter initialization."""
    for param_name in sim_robot_parameter_names:
        assert test_sim_robot.get_parameter(param_name).value == ""


def test_sim_robot_parameter_setting(
    configured_sim_robot: TestObeliskSimRobot, sim_robot_parameter_names: List[str]
) -> None:
    """Test parameter setting."""
    try:
        configured_sim_robot.get_parameters(sim_robot_parameter_names)
    except rclpy.exceptions.ParameterUninitializedException as e:
        pytest.fail(f"Failed to access parameters after setting them: {e}")


def test_sim_robot_configuration(
    configured_sim_robot: TestObeliskSimRobot,
    check_node_attributes: Callable[[Any, List[str], bool], None],
    sim_robot_parameter_names: List[str],
) -> None:
    """Test configuration."""
    component_names = [
        "test_cbg",
        "subscriber_ctrl",
        "timer_true_sim_state",
        "publisher_true_sim_state",
    ]
    check_node_attributes(configured_sim_robot, sim_robot_parameter_names + component_names, should_exist=True)


def test_sim_robot_functionality(configured_sim_robot: TestObeliskSimRobot) -> None:
    """Test the functionality of the ObeliskSimRobot."""
    configured_sim_robot.apply_control(ocm.PositionSetpoint())
    true_sim_state = configured_sim_robot.publish_true_sim_state()
    assert isinstance(true_sim_state, osm.TrueSimState)

    configured_sim_robot.on_activate(configured_sim_robot._state_machine.current_state)


def test_sim_robot_on_configure_success(configured_sim_robot: TestObeliskSimRobot) -> None:
    """Test successful configuration of the sim robot."""
    result = configured_sim_robot.on_configure(None)
    assert result == TransitionCallbackReturn.SUCCESS
    assert hasattr(configured_sim_robot, "subscriber_ctrl")
    assert hasattr(configured_sim_robot, "timer_true_sim_state")
    assert hasattr(configured_sim_robot, "publisher_true_sim_state")


def test_sim_robot_on_configure_missing_parameters(test_sim_robot: TestObeliskSimRobot) -> None:
    """Test configuration with missing parameters."""
    with pytest.raises(ParameterUninitializedException):
        test_sim_robot.on_configure(None)


def test_sim_robot_on_cleanup(configured_sim_robot: TestObeliskSimRobot) -> None:
    """Test cleanup of the sim robot."""
    result = configured_sim_robot.on_cleanup(None)
    assert result == TransitionCallbackReturn.SUCCESS


def test_sim_robot_true_sim_state_callback(configured_sim_robot: TestObeliskSimRobot) -> None:
    """Test that the publish_true_sim_state method is set as the timer callback."""
    assert configured_sim_robot.timer_true_sim_state.callback == configured_sim_robot.publish_true_sim_state


def test_sim_robot_run_simulator(configured_sim_robot: TestObeliskSimRobot) -> None:
    """Test the run_simulator method."""
    configured_sim_robot.run_simulator()
    assert configured_sim_robot.test_run_simulator_flag


@pytest.mark.parametrize(
    "invalid_config",
    [
        {"timer_true_sim_state_setting": "invalid_setting"},
        {"pub_true_sim_state_setting": "invalid_setting"},
    ],
)
def test_sim_robot_invalid_configuration(test_sim_robot: TestObeliskSimRobot, invalid_config: Dict[str, Any]) -> None:
    """Test handling of invalid configuration for sim robot."""
    for key, value in invalid_config.items():
        test_sim_robot.set_parameters([rclpy.Parameter(key, value=value)])
    with pytest.raises(Exception) as exc_info:  # The exact exception type may vary based on implementation
        test_sim_robot.on_configure(None)
    assert isinstance(exc_info.value, Exception)


def test_sim_robot_lifecycle_transitions(configured_sim_robot: TestObeliskSimRobot) -> None:
    """Test full lifecycle transitions of the sim robot."""
    assert configured_sim_robot.on_configure(None) == TransitionCallbackReturn.SUCCESS
    assert configured_sim_robot.on_activate(None) == TransitionCallbackReturn.SUCCESS
    assert configured_sim_robot.on_deactivate(None) == TransitionCallbackReturn.SUCCESS
    assert configured_sim_robot.on_cleanup(None) == TransitionCallbackReturn.SUCCESS

    assert configured_sim_robot.on_configure(None) == TransitionCallbackReturn.SUCCESS
    assert configured_sim_robot.on_shutdown(None) == TransitionCallbackReturn.SUCCESS


def test_sim_robot_inheritance(configured_sim_robot: TestObeliskSimRobot) -> None:
    """Test that ObeliskSimRobot correctly inherits from ObeliskRobot."""
    assert isinstance(configured_sim_robot, ObeliskRobot)
