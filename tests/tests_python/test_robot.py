from typing import Any, Callable, Dict, Generator, List

import obelisk_control_msgs.msg as ocm
import obelisk_sensor_msgs.msg as osm
import pytest
import rclpy
from rclpy.exceptions import ParameterUninitializedException
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription

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


def test_on_configure_success(configured_robot: TestObeliskRobot) -> None:
    """Test successful configuration of the robot."""
    result = configured_robot.on_configure(None)
    assert result == TransitionCallbackReturn.SUCCESS
    assert hasattr(configured_robot, "subscriber_ctrl")
    assert isinstance(configured_robot.subscriber_ctrl, Subscription)
    assert len(configured_robot.publisher_sensors) == 2  # noqa: PLR2004
    assert all(isinstance(pub, Publisher) for pub in configured_robot.publisher_sensors)


def test_on_configure_missing_parameters(test_robot: TestObeliskRobot) -> None:
    """Test configuration with missing parameters."""
    with pytest.raises(ParameterUninitializedException):
        test_robot.on_configure(None)


def test_on_cleanup(configured_robot: TestObeliskRobot) -> None:
    """Test cleanup of the robot."""
    result = configured_robot.on_cleanup(None)
    assert result == TransitionCallbackReturn.SUCCESS
    assert not hasattr(configured_robot, "subscriber_ctrl")
    assert not hasattr(configured_robot, "publisher_sensors")
    assert not hasattr(configured_robot, "sub_ctrl_config_str")
    assert not hasattr(configured_robot, "pub_sensor_config_strs")


def test_apply_control_callback(configured_robot: TestObeliskRobot) -> None:
    """Test that the apply_control method is set as the subscriber callback."""
    assert configured_robot.subscriber_ctrl.callback == configured_robot.apply_control


def test_publisher_creation(configured_robot: TestObeliskRobot) -> None:
    """Test creation of publishers based on configuration strings."""
    assert len(configured_robot.publisher_sensors) == 2  # noqa: PLR2004
    assert all(isinstance(pub, Publisher) for pub in configured_robot.publisher_sensors)


@pytest.mark.parametrize(
    "invalid_config",
    [
        {"sub_ctrl_config_str": "invalid_config_string"},
        {"pub_sensor_config_strs": ["invalid_config_string"]},
    ],
)
def test_invalid_configuration(test_robot: TestObeliskRobot, invalid_config: Dict[str, Any]) -> None:
    """Test handling of invalid configuration strings."""
    for key, value in invalid_config.items():
        test_robot.set_parameters([rclpy.Parameter(key, value=value)])
    with pytest.raises(Exception) as exc_info:  # The exact exception type may vary based on implementation
        test_robot.on_configure(None)
    assert isinstance(exc_info.value, Exception)


def test_lifecycle_transitions(configured_robot: TestObeliskRobot) -> None:
    """Test full lifecycle transitions of the robot."""
    assert configured_robot.on_configure(None) == TransitionCallbackReturn.SUCCESS
    assert configured_robot.on_cleanup(None) == TransitionCallbackReturn.SUCCESS

    assert configured_robot.on_configure(None) == TransitionCallbackReturn.SUCCESS
    assert configured_robot.on_shutdown(None) == TransitionCallbackReturn.SUCCESS


def test_inheritance(configured_robot: TestObeliskRobot) -> None:
    """Test that ObeliskRobot correctly inherits from ObeliskNode."""
    from obelisk_py.node import ObeliskNode

    assert isinstance(configured_robot, ObeliskNode)
