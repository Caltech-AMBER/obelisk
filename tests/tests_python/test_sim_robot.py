import time
from typing import Any, Callable, Dict, Generator, List

import numpy as np
import obelisk_control_msgs.msg as ocm
import obelisk_sensor_msgs.msg as osm
import pytest
import rclpy

from obelisk_py.obelisk_typing import ObeliskControlMsg
from obelisk_py.robot import ObeliskSimRobot

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
        self._set_shared_ctrl([0.1, 0.2])


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
        "callback_group_config_strs": ["test_cbg:ReentrantCallbackGroup"],
        "sub_ctrl_config_str": (
            "msg_type:PositionSetpoint,"
            "topic:/obelisk/test_sim_robot/ctrl,"
            "callback:apply_control,"
            "history_depth:10,"
            "callback_group:None,"
            "non_obelisk:False"
        ),
        "pub_sensor_config_strs": [
            (
                "msg_type:JointEncoder,"
                "topic:/obelisk/test_sim_robot/sensor1,"
                "history_depth:10,"
                "callback_group:None,"
                "non_obelisk:False"
            ),
            (
                "msg_type:JointEncoder,"
                "topic:/obelisk/test_sim_robot/sensor2,"
                "history_depth:10,"
                "callback_group:None,"
                "non_obelisk:False"
            ),
        ],
        "n_u": 2,
        "timer_true_sim_state_config_str": (
            "timer_period_sec:0.1," "callback:publish_true_sim_state," "callback_group:None"
        ),
        "pub_true_sim_state_config_str": (
            "msg_type:TrueSimState,"
            "topic:/obelisk/test_sim_robot/true_sim_state,"
            "history_depth:10,"
            "callback_group:None,"
            "non_obelisk:False"
        ),
    }
    set_node_parameters(test_sim_robot, parameter_dict)
    test_sim_robot.on_configure(test_sim_robot._state_machine.current_state)
    return test_sim_robot


@pytest.fixture
def sim_robot_parameter_names() -> List[str]:
    """Parameter names for the ObeliskSimRobot class."""
    return [
        "n_u",
        "timer_true_sim_state_config_str",
        "pub_true_sim_state_config_str",
    ]


# ##### #
# TESTS #
# ##### #


def test_sim_robot_parameter_initialization(
    test_sim_robot: TestObeliskSimRobot, sim_robot_parameter_names: List[str]
) -> None:
    """Test parameter initialization."""
    with pytest.raises(rclpy.exceptions.ParameterUninitializedException):
        test_sim_robot.get_parameters(sim_robot_parameter_names)


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
        "publisher_sensors",
        "timer_true_sim_state",
        "publisher_true_sim_state",
    ]
    check_node_attributes(configured_sim_robot, sim_robot_parameter_names + component_names, should_exist=True)


def test_sim_robot_cleanup(
    configured_sim_robot: TestObeliskSimRobot,
    check_node_attributes: Callable[[Any, List[str], bool], None],
    sim_robot_parameter_names: List[str],
) -> None:
    """Test cleanup."""
    component_names = [
        "test_cbg",
        "subscriber_ctrl",
        "publisher_sensors",
        "timer_true_sim_state",
        "publisher_true_sim_state",
    ]
    configured_sim_robot.on_cleanup(configured_sim_robot._state_machine.current_state)
    check_node_attributes(configured_sim_robot, sim_robot_parameter_names + component_names, should_exist=False)


def test_sim_robot_functionality(configured_sim_robot: TestObeliskSimRobot) -> None:
    """Test the functionality of the ObeliskSimRobot."""
    configured_sim_robot.apply_control(ocm.PositionSetpoint())
    true_sim_state = configured_sim_robot.publish_true_sim_state()
    assert isinstance(true_sim_state, osm.TrueSimState)

    configured_sim_robot.on_activate(configured_sim_robot._state_machine.current_state)
    time.sleep(0.001)
    assert np.allclose(list(configured_sim_robot.shared_ctrl), [0.1, 0.2])
