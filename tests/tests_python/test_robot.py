from typing import Any, Callable, Dict, Generator, List

import obelisk_control_msgs.msg as ocm
import obelisk_sensor_msgs.msg as osm
import pytest
import rclpy

from obelisk_py.obelisk_typing import ObeliskControlMsg, ObeliskMsg, ObeliskSensorMsg, is_in_bound
from obelisk_py.robot import ObeliskRobot

# ##### #
# SETUP #
# ##### #


class TestObeliskRobot(ObeliskRobot):
    """Test ObeliskRobot class."""

    def apply_control(self, _: ObeliskControlMsg) -> None:
        """Apply control."""
        pass

    def publish_measurement1(self) -> ObeliskSensorMsg:
        """Publish measurement 1."""
        return osm.JointEncoder()

    def publish_measurement2(self) -> ObeliskSensorMsg:
        """Publish measurement 2."""
        return osm.JointEncoder()


@pytest.fixture
def test_robot(ros_context: Any) -> Generator[TestObeliskRobot, None, None]:
    """Fixture for the TestObeliskRobot class."""
    robot = TestObeliskRobot("test_robot")
    yield robot
    robot.destroy_node()


@pytest.fixture
def configured_robot(
    test_robot: TestObeliskRobot, set_node_parameters: Callable[[Any, Dict], None]
) -> TestObeliskRobot:
    """Fixture for the TestObeliskRobot class with parameters set."""
    parameter_dict = {
        "callback_group_config_strs": ["test_cbg:ReentrantCallbackGroup"],
        "sub_ctrl_config_str": (
            "msg_type:PositionSetpoint,"
            "topic:/obelisk/test_robot/ctrl,"
            "callback:apply_control,"
            "history_depth:10,"
            "callback_group:None,"
            "non_obelisk:False"
        ),
        "pub_sensor_config_strs": [
            (
                "msg_type:JointEncoder,"
                "topic:/obelisk/test_robot/sensor1,"
                "history_depth:10,"
                "callback_group:None,"
                "non_obelisk:False"
            ),
            (
                "msg_type:JointEncoder,"
                "topic:/obelisk/test_robot/sensor2,"
                "history_depth:10,"
                "callback_group:None,"
                "non_obelisk:False"
            ),
        ],
    }
    set_node_parameters(test_robot, parameter_dict)
    test_robot.on_configure(test_robot._state_machine.current_state)
    return test_robot


@pytest.fixture
def robot_parameter_names() -> List[str]:
    """Return the parameter names for the robot."""
    return ["sub_ctrl_config_str", "pub_sensor_config_strs"]


# ##### #
# TESTS #
# ##### #


def test_robot_parameter_initialization(test_robot: TestObeliskRobot, robot_parameter_names: List[str]) -> None:
    """Test parameter initialization."""
    with pytest.raises(rclpy.exceptions.ParameterUninitializedException):
        test_robot.get_parameters(robot_parameter_names)


def test_robot_parameter_setting(configured_robot: TestObeliskRobot, robot_parameter_names: List[str]) -> None:
    """Test parameter setting."""
    try:
        configured_robot.get_parameters(robot_parameter_names)
    except rclpy.exceptions.ParameterUninitializedException as e:
        pytest.fail(f"Failed to access parameters after setting them: {e}")


def test_robot_configuration(
    configured_robot: TestObeliskRobot,
    check_node_attributes: Callable[[Any, List[str], bool], None],
    robot_parameter_names: List[str],
) -> None:
    """Test node configuration."""
    component_names = ["test_cbg", "subscriber_ctrl", "publisher_sensors"]
    check_node_attributes(configured_robot, robot_parameter_names + component_names, should_exist=True)


def test_robot_cleanup(
    configured_robot: TestObeliskRobot,
    check_node_attributes: Callable[[Any, List[str], bool], None],
    robot_parameter_names: List[str],
) -> None:
    """Test node cleanup."""
    component_names = ["test_cbg", "subscriber_ctrl", "publisher_sensors"]
    configured_robot.on_cleanup(configured_robot._state_machine.current_state)
    check_node_attributes(configured_robot, robot_parameter_names + component_names, should_exist=False)


def test_robot_functionality(configured_robot: TestObeliskRobot) -> None:
    """Test robot functionality."""
    configured_robot.apply_control(ocm.PositionSetpoint())

    obk_sensor_msg1 = configured_robot.publish_measurement1()
    obk_sensor_msg2 = configured_robot.publish_measurement2()

    for msg in [obk_sensor_msg1, obk_sensor_msg2]:
        assert isinstance(msg, osm.JointEncoder)
        assert is_in_bound(type(msg), ObeliskSensorMsg)
        assert is_in_bound(type(msg), ObeliskMsg)
