from typing import Any, Callable, Generator

import obelisk_sensor_msgs.msg as osm
import pytest
from rclpy.lifecycle.node import TransitionCallbackReturn
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.timer import Timer

from obelisk_py.core.obelisk_typing import ObeliskControlMsg
from obelisk_py.core.robot import ObeliskRobot, ObeliskSimRobot

# ##### #
# SETUP #
# ##### #


class TestRobot(ObeliskRobot):
    """A concrete implementation of ObeliskRobot for testing purposes."""

    def apply_control(self, control_msg: ObeliskControlMsg) -> None:
        """Apply the control message to the robot."""
        pass


class TestSimRobot(ObeliskSimRobot):
    """A concrete implementation of ObeliskSimRobot for testing purposes."""

    def apply_control(self, control_msg: ObeliskControlMsg) -> None:
        """Apply the control message to the robot."""
        pass

    def run_simulator(self) -> None:
        """Run the simulator."""
        pass


@pytest.fixture
def test_robot(ros_context: Any) -> Generator[TestRobot, None, None]:
    """Fixture for the TestRobot class."""
    robot = TestRobot("test_robot")
    yield robot
    robot.destroy_node()


@pytest.fixture
def test_sim_robot(ros_context: Any) -> Generator[TestSimRobot, None, None]:
    """Fixture for the TestSimRobot class."""
    sim_robot = TestSimRobot("test_sim_robot")
    yield sim_robot
    sim_robot.destroy_node()


# ##### #
# TESTS #
# ##### #


def test_robot_initialization(test_robot: TestRobot) -> None:
    """Test the initialization of the ObeliskRobot.

    This test checks if the robot is properly initialized with the correct name and if the required subscription
    is registered.

    Parameters:
        test_robot: An instance of TestRobot.
    """
    assert test_robot.get_name() == "test_robot"
    assert test_robot.has_parameter("sub_ctrl_setting")


def test_robot_subscription_registration(test_robot: TestRobot) -> None:
    """Test the registration of the control subscription.

    This test verifies that the control subscription is properly registered with the correct key, callback,
    and message type.

    Parameters:
        test_robot: An instance of TestRobot.
    """
    sub_setting = next(s for s in test_robot._obk_sub_settings if s["key"] == "subscriber_ctrl")
    assert sub_setting["callback"] == test_robot.apply_control
    assert sub_setting["msg_type"] is None  # Should be specified in config file


def test_robot_configuration(test_robot: TestRobot, set_node_parameters: Callable) -> None:
    """Test the configuration of the ObeliskRobot.

    This test verifies that the robot can be properly configured with the necessary parameters and that the
    subscription is created correctly.

    Parameters:
        test_robot: An instance of TestRobot.
        set_node_parameters: A fixture to set node parameters.
    """
    set_node_parameters(test_robot, {"sub_ctrl_setting": "topic:/test_control,msg_type:PositionSetpoint"})
    result = test_robot.on_configure(None)

    assert result == TransitionCallbackReturn.SUCCESS
    assert "subscriber_ctrl" in test_robot.obk_subscriptions
    assert isinstance(test_robot.obk_subscriptions["subscriber_ctrl"], Subscription)


def test_sim_robot_initialization(test_sim_robot: TestSimRobot) -> None:
    """Test the initialization of the ObeliskSimRobot.

    This test checks if the sim robot is properly initialized with the correct name and if the required timer
    and publisher are registered.

    Parameters:
        test_sim_robot: An instance of TestSimRobot.
    """
    assert test_sim_robot.get_name() == "test_sim_robot"
    assert test_sim_robot.has_parameter("timer_true_sim_state_setting")
    assert test_sim_robot.has_parameter("pub_true_sim_state_setting")


def test_sim_robot_timer_registration(test_sim_robot: TestSimRobot) -> None:
    """Test the registration of the true sim state timer.

    This test verifies that the true sim state timer is properly registered with the correct key and callback.

    Parameters:
        test_sim_robot: An instance of TestSimRobot.
    """
    timer_setting = next(s for s in test_sim_robot._obk_timer_settings if s["key"] == "timer_true_sim_state")
    assert timer_setting["callback"] == test_sim_robot.publish_true_sim_state


def test_sim_robot_publisher_registration(test_sim_robot: TestSimRobot) -> None:
    """Test the registration of the true sim state publisher.

    This test verifies that the true sim state publisher is properly registered with the correct key and message type.

    Parameters:
        test_sim_robot: An instance of TestSimRobot.
    """
    pub_setting = next(s for s in test_sim_robot._obk_pub_settings if s["key"] == "publisher_true_sim_state")
    assert pub_setting["msg_type"] == osm.TrueSimState


def test_sim_robot_configuration(test_sim_robot: TestSimRobot, set_node_parameters: Callable) -> None:
    """Test the configuration of the ObeliskSimRobot.

    This test verifies that the sim robot can be properly configured with the necessary parameters and that the
    timer and publisher are created correctly.

    Parameters:
        test_sim_robot: An instance of TestSimRobot.
        set_node_parameters: A fixture to set node parameters.
    """
    set_node_parameters(
        test_sim_robot,
        {
            "sub_ctrl_setting": "topic:/test_control,msg_type:PositionSetpoint",
            "timer_true_sim_state_setting": "timer_period_sec:0.1",
            "pub_true_sim_state_setting": "topic:/test_true_sim_state",
        },
    )
    result = test_sim_robot.on_configure(None)

    assert result == TransitionCallbackReturn.SUCCESS
    assert "timer_true_sim_state" in test_sim_robot.obk_timers
    assert isinstance(test_sim_robot.obk_timers["timer_true_sim_state"], Timer)
    assert "publisher_true_sim_state" in test_sim_robot.obk_publishers
    assert isinstance(test_sim_robot.obk_publishers["publisher_true_sim_state"], Publisher)


def test_abstract_methods() -> None:
    """Test the abstract methods of ObeliskRobot and ObeliskSimRobot.

    This test verifies that the abstract methods are properly defined and must be implemented by subclasses.
    """
    with pytest.raises(TypeError):

        class IncompleteRobot(ObeliskRobot):
            pass

        IncompleteRobot("incomplete_robot")

    with pytest.raises(TypeError):

        class IncompleteSimRobot(ObeliskSimRobot):
            def apply_control(self, control_msg: ObeliskControlMsg) -> None:
                pass

        IncompleteSimRobot("incomplete_sim_robot")

    class CompleteRobot(ObeliskRobot):
        def apply_control(self, control_msg: ObeliskControlMsg) -> None:
            pass

    complete_robot = CompleteRobot("complete_robot")
    assert hasattr(complete_robot, "apply_control")

    class CompleteSimRobot(ObeliskSimRobot):
        def apply_control(self, control_msg: ObeliskControlMsg) -> None:
            pass

        def run_simulator(self) -> None:
            pass

    complete_sim_robot = CompleteSimRobot("complete_sim_robot")
    assert hasattr(complete_sim_robot, "apply_control")
    assert hasattr(complete_sim_robot, "run_simulator")
