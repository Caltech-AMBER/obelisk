"""Tests for launch_utils (post-config-string-refactor).

The producer now bundles all Obelisk-internal sections (publishers/subscribers/timers/sim/
callback_groups) into a single ``obelisk_settings`` ROS parameter whose value is a YAML string.
User ``params`` and ``params_path`` flow through as separate ROS parameters.
"""

from pathlib import Path
from typing import Any, Dict

import pytest
import yaml
from launch_ros.actions import LifecycleNode

from obelisk_py.core.utils.launch_utils import (
    get_launch_actions_from_node_settings,
    get_parameters_dict,
    load_config_file,
)


@pytest.fixture
def test_config() -> Dict[str, Any]:
    """Fixture to provide test configuration data (mirrors test_assets/test_config.yaml)."""
    return {
        "onboard": {
            "control": [
                {
                    "pkg": "test_pkg1",
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
                }
            ],
            "estimation": [
                {
                    "pkg": "test_pkg2",
                    "executable": "test_estimator",
                    "callback_groups": {"cbg1": "ReentrantCallbackGroup"},
                }
            ],
            "robot": [
                {
                    "pkg": "test_pkg3",
                    "executable": "test_robot",
                    "callback_groups": {"cbg1": "MutuallyExclusiveCallbackGroup"},
                }
            ],
            "sensing": [
                {
                    "pkg": "test_pkg4",
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
                    "pkg": "test_pkg5",
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


@pytest.fixture
def test_global_state_node() -> LifecycleNode:
    """Fixture to provide a dummy global state node."""
    return LifecycleNode(
        namespace="",
        package="obelisk_ros",
        executable="global_state",
        name="global_state",
        output="screen",
    )


def test_load_config_file(test_config: Dict[str, Any]) -> None:
    """load_config_file reads YAML files (used by the launch entry point)."""
    abs_path = Path(__file__).parent.parent.parent / "test_assets" / "test_config.yaml"
    result = load_config_file(abs_path)
    assert result == test_config

    with pytest.raises(FileNotFoundError):
        load_config_file("non_existent.yaml")


def test_get_parameters_dict_bundles_obelisk_sections(test_config: Dict[str, Any]) -> None:
    """A controller node's settings collapse into a single obelisk_settings YAML string."""
    node_settings = test_config["onboard"]["control"][0]
    parameters_dict = get_parameters_dict(node_settings)

    assert "obelisk_settings" in parameters_dict
    bundled = yaml.safe_load(parameters_dict["obelisk_settings"])

    # callback_groups round-trip
    assert bundled["callback_groups"] == {
        "cbg1": "MutuallyExclusiveCallbackGroup",
        "cbg2": "ReentrantCallbackGroup",
    }
    # publisher entry has key + topic + ros_parameter dropped
    assert len(bundled["publishers"]) == 1
    pub = bundled["publishers"][0]
    assert pub["key"] == "pub1"
    assert pub["topic"] == "/test/controller/pub1"
    assert "ros_parameter" not in pub
    # timer + subscriber too
    assert bundled["timers"][0]["timer_period_sec"] == 0.1
    assert bundled["subscribers"][0]["callback_group"] == "cbg2"


def test_get_parameters_dict_derives_key_from_ros_parameter() -> None:
    """An entry that has only ``ros_parameter`` (no ``key``) gets ``key`` derived by stripping ``_setting``."""
    node_settings = {
        "publishers": [
            {"ros_parameter": "pub_ctrl_setting", "topic": "/foo"},
        ]
    }
    bundled = yaml.safe_load(get_parameters_dict(node_settings)["obelisk_settings"])
    assert bundled["publishers"][0]["key"] == "pub_ctrl"


def test_get_parameters_dict_user_params_passthrough() -> None:
    """User-defined ``params`` and ``params_path`` are forwarded as separate ROS parameters."""
    node_settings = {
        "params_path": "/some/path.txt",
        "params": {"user_int": 7, "user_str": "x"},
    }
    parameters_dict = get_parameters_dict(node_settings)
    assert parameters_dict["params_path"] == "/some/path.txt"
    assert parameters_dict["user_int"] == 7
    assert parameters_dict["user_str"] == "x"


def test_get_parameters_dict_empty_node() -> None:
    """A node with only callback_groups still produces obelisk_settings."""
    minimal = {"pkg": "p", "executable": "e", "callback_groups": {"cbg1": "MutuallyExclusiveCallbackGroup"}}
    parameters_dict = get_parameters_dict(minimal)
    bundled = yaml.safe_load(parameters_dict["obelisk_settings"])
    assert bundled == {"callback_groups": {"cbg1": "MutuallyExclusiveCallbackGroup"}}


def test_get_parameters_dict_handles_none() -> None:
    """get_parameters_dict on None returns an empty dict."""
    assert get_parameters_dict(None) == {}


def test_get_launch_actions_from_node_settings(
    test_config: Dict[str, Any], test_global_state_node: LifecycleNode
) -> None:
    """Each node setting becomes one LifecycleNode plus its lifecycle handlers (8 actions per node)."""
    node_settings = test_config["onboard"]["control"]
    launch_actions = get_launch_actions_from_node_settings(node_settings, "control", test_global_state_node)
    assert len(launch_actions) == 8  # noqa: PLR2004
    assert isinstance(launch_actions[0], LifecycleNode)


def test_get_launch_actions_from_node_settings_invalid_type(test_global_state_node: LifecycleNode) -> None:
    """An unknown node type asserts."""
    invalid = {
        "pkg": "p",
        "executable": "e",
        "callback_groups": {"cbg1": "MutuallyExclusiveCallbackGroup"},
    }
    with pytest.raises(AssertionError):
        get_launch_actions_from_node_settings(invalid, "invalid_type", test_global_state_node)
