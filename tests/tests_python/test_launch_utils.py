from pathlib import Path
from typing import Any, Dict

import pytest
from launch_ros.actions import LifecycleNode

from obelisk_py.utils.launch import (
    get_component_settings_subdict,
    get_launch_actions_from_node_settings,
    get_parameters_dict,
    load_config_file,
)


@pytest.fixture
def test_config() -> Dict[str, Any]:
    """Fixture to provide test configuration data."""
    return {
        "onboard": {
            "controller": {
                "impl": "python",
                "executable": "test_controller",
                "callback_groups": {"cbg1": "MutuallyExclusiveCallbackGroup", "cbg2": "ReentrantCallbackGroup"},
                "publishers": [
                    {
                        "ros_parameter": "pub_ctrl_settings1",
                        "key": "pub1",
                        "topic": "/test/controller/pub1",
                        "msg_type": "TestMsg",
                        "history_depth": 10,
                        "callback_group": "cbg1",
                        "non_obelisk": False,
                    }
                ],
                "subscribers": [
                    {
                        "ros_parameter": "sub_ctrl_settings1",
                        "key": "sub1",
                        "topic": "/test/controller/sub1",
                        "msg_type": "TestMsg",
                        "history_depth": 5,
                        "callback_key": "sub_callback1",
                        "callback_group": "cbg2",
                        "non_obelisk": False,
                    }
                ],
                "timers": [
                    {
                        "ros_parameter": "timer_ctrl_settings1",
                        "key": "timer1",
                        "timer_period_sec": 0.1,
                        "callback_group": "cbg1",
                        "callback_key": "timer_callback1",
                    }
                ],
            },
            "estimator": {
                "impl": "cpp",
                "executable": "test_estimator",
                "callback_groups": {"cbg1": "ReentrantCallbackGroup"},
            },
            "robot": {
                "impl": "python",
                "executable": "test_robot",
                "callback_groups": {"cbg1": "MutuallyExclusiveCallbackGroup"},
            },
            "sensors": [
                {
                    "impl": "python",
                    "executable": "test_sensor1",
                    "callback_groups": {"cbg1": "MutuallyExclusiveCallbackGroup"},
                    "publishers": [
                        {
                            "ros_parameter": "pub_sensor_settings1",
                            "key": "pub1",
                            "topic": "/test/sensor1/pub1",
                            "msg_type": "SensorMsg",
                            "history_depth": 10,
                            "callback_group": "cbg1",
                            "non_obelisk": False,
                        }
                    ],
                },
                {
                    "impl": "cpp",
                    "executable": "test_sensor2",
                    "callback_groups": {"cbg1": "ReentrantCallbackGroup"},
                    "timers": [
                        {
                            "ros_parameter": "timer_sensor_settings1",
                            "key": "timer1",
                            "timer_period_sec": 0.01,
                            "callback_group": "cbg1",
                            "callback_key": "timer_callback1",
                        }
                    ],
                },
            ],
        }
    }


def test_load_config_file(test_config: Dict[str, Any]) -> None:
    """Test the load_config_file function.

    Args:
        test_config: Test configuration fixture.
    """
    # Test with absolute path
    abs_path = Path(__file__).parent / "test_assets" / "test_config.yaml"
    result = load_config_file(abs_path)
    assert result == test_config

    # Test file not found
    with pytest.raises(FileNotFoundError):
        load_config_file("non_existent.yaml")


def test_get_component_settings_subdict(test_config: Dict[str, Any]) -> None:
    """Test the get_component_settings_subdict function.

    Args:
        test_config: Test configuration fixture.
    """
    node_settings = test_config["onboard"]["controller"]

    # Test publishers
    pub_settings = get_component_settings_subdict(node_settings, "publishers")
    expected_pub_settings = {
        "pub_ctrl_settings1": (
            "key:pub1,topic:/test/controller/pub1,msg_type:TestMsg,history_depth:10,"
            "callback_group:cbg1,non_obelisk:False"
        )
    }
    assert pub_settings == expected_pub_settings

    # Test subscribers
    sub_settings = get_component_settings_subdict(node_settings, "subscribers")
    expected_sub_settings = {
        "sub_ctrl_settings1": (
            "key:sub1,topic:/test/controller/sub1,msg_type:TestMsg,history_depth:5,callback_key:sub_callback1,"
            "callback_group:cbg2,non_obelisk:False"
        )
    }
    assert sub_settings == expected_sub_settings

    # Test timers
    timer_settings = get_component_settings_subdict(node_settings, "timers")
    expected_timer_settings = {
        "timer_ctrl_settings1": "key:timer1,timer_period_sec:0.1,callback_group:cbg1,callback_key:timer_callback1"
    }
    assert timer_settings == expected_timer_settings


def test_get_parameters_dict(test_config: Dict[str, Any]) -> None:
    """Test the get_parameters_dict function.

    Args:
        test_config: Test configuration fixture.
    """
    node_settings = test_config["onboard"]["controller"]
    parameters_dict = get_parameters_dict(node_settings)

    expected_dict = {
        "callback_group_settings": "cbg1:MutuallyExclusiveCallbackGroup,cbg2:ReentrantCallbackGroup",
        "pub_ctrl_settings1": (
            "key:pub1,topic:/test/controller/pub1,msg_type:TestMsg,history_depth:10,callback_group:cbg1,"
            "non_obelisk:False"
        ),
        "sub_ctrl_settings1": (
            "key:sub1,topic:/test/controller/sub1,msg_type:TestMsg,history_depth:5,callback_key:sub_callback1,"
            "callback_group:cbg2,non_obelisk:False"
        ),
        "timer_ctrl_settings1": "key:timer1,timer_period_sec:0.1,callback_group:cbg1,callback_key:timer_callback1",
    }

    assert parameters_dict == expected_dict


def test_get_launch_actions_from_node_settings(test_config: Dict[str, Any]) -> None:
    """Test the get_launch_actions_from_node_settings function.

    Args:
        test_config: Test configuration fixture.
    """
    node_settings = test_config["onboard"]["controller"]

    launch_actions = get_launch_actions_from_node_settings(node_settings, "controller")

    assert len(launch_actions) == 1
    assert isinstance(launch_actions[0], LifecycleNode)

    # Test for sensors (multiple nodes)
    sensors_settings = [
        {
            "impl": "python",
            "executable": "test_sensor1",
            "callback_groups": {"cbg1": "MutuallyExclusiveCallbackGroup"},
            "publishers": [
                {
                    "ros_parameter": "pub_sensor_settings1",
                    "key": "pub1",
                    "topic": "/test/sensor1/pub1",
                    "msg_type": "SensorMsg",
                    "history_depth": 10,
                    "callback_group": "cbg1",
                    "non_obelisk": False,
                }
            ],
        },
        {
            "impl": "cpp",
            "executable": "test_sensor2",
            "callback_groups": {"cbg1": "ReentrantCallbackGroup"},
            "timers": [
                {
                    "ros_parameter": "timer_sensor_settings1",
                    "key": "timer1",
                    "timer_period_sec": 0.01,
                    "callback_group": "cbg1",
                    "callback_key": "timer_callback1",
                }
            ],
        },
    ]

    launch_actions = get_launch_actions_from_node_settings(sensors_settings, "sensors")

    assert len(launch_actions) == 2  # noqa: PLR2004
    assert isinstance(launch_actions[0], LifecycleNode)
    assert isinstance(launch_actions[1], LifecycleNode)


def test_get_component_settings_subdict_sensors(test_config: Dict[str, Any]) -> None:
    """Test the get_component_settings_subdict function for sensors.

    Args:
        test_config: Test configuration fixture.
    """
    sensors_settings = test_config["onboard"]["sensors"]

    # Test publishers for the first sensor
    pub_settings = get_component_settings_subdict(sensors_settings[0], "publishers")
    expected_pub_settings = {
        "pub_sensor_settings1": (
            "key:pub1,topic:/test/sensor1/pub1,msg_type:SensorMsg,history_depth:10,callback_group:cbg1,"
            "non_obelisk:False"
        )
    }
    assert pub_settings == expected_pub_settings

    # Test timers for the second sensor
    timer_settings = get_component_settings_subdict(sensors_settings[1], "timers")
    expected_timer_settings = {
        "timer_sensor_settings1": "key:timer1,timer_period_sec:0.01,callback_group:cbg1,callback_key:timer_callback1"
    }
    assert timer_settings == expected_timer_settings


def test_get_parameters_dict_sensors(test_config: Dict[str, Any]) -> None:
    """Test the get_parameters_dict function for sensors.

    Args:
        test_config: Test configuration fixture.
    """
    sensors_settings = test_config["onboard"]["sensors"]

    # Test parameters for the first sensor
    parameters_dict1 = get_parameters_dict(sensors_settings[0])
    expected_dict1 = {
        "callback_group_settings": "cbg1:MutuallyExclusiveCallbackGroup",
        "pub_sensor_settings1": (
            "key:pub1,topic:/test/sensor1/pub1,msg_type:SensorMsg,history_depth:10,callback_group:cbg1,"
            "non_obelisk:False"
        ),
    }
    assert parameters_dict1 == expected_dict1

    # Test parameters for the second sensor
    parameters_dict2 = get_parameters_dict(sensors_settings[1])
    expected_dict2 = {
        "callback_group_settings": "cbg1:ReentrantCallbackGroup",
        "timer_sensor_settings1": "key:timer1,timer_period_sec:0.01,callback_group:cbg1,callback_key:timer_callback1",
    }
    assert parameters_dict2 == expected_dict2


def test_get_parameters_dict_empty_node(test_config: Dict[str, Any]) -> None:
    """Test the get_parameters_dict function for a node with minimal settings.

    Args:
        test_config: Test configuration fixture.
    """
    minimal_node_settings = {
        "impl": "python",
        "executable": "test_minimal",
        "callback_groups": {"cbg1": "MutuallyExclusiveCallbackGroup"},
    }
    parameters_dict = get_parameters_dict(minimal_node_settings)
    expected_dict = {"callback_group_settings": "cbg1:MutuallyExclusiveCallbackGroup"}
    assert parameters_dict == expected_dict


def test_get_launch_actions_from_node_settings_invalid_type() -> None:
    """Test the get_launch_actions_from_node_settings function with an invalid node type."""
    invalid_node_settings = {
        "impl": "python",
        "executable": "test_invalid",
        "callback_groups": {"cbg1": "MutuallyExclusiveCallbackGroup"},
    }

    with pytest.raises(AssertionError):
        get_launch_actions_from_node_settings(invalid_node_settings, "invalid_type")
