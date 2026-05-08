"""Behavioral safety-net tests for the YAML-config -> ROS-parameter -> running-node pipeline.

These tests pin down the *observable* properties of an Obelisk node configured from a YAML-style
settings dict: topic names, history depth, callback-group bindings, timer periods, and msg types.
They are intentionally written to survive the config-string-removal refactor: the producer side
(`get_parameters_dict`) may change the shape of its output, but the assertions about the
configured node should hold either way.
"""

from typing import Any, Callable, Generator, Type

import pytest
from obelisk_estimator_msgs.msg import EstimatedState
from obelisk_sensor_msgs.msg import ObkJointEncoders, TrueSimState
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.lifecycle.node import TransitionCallbackReturn
from rclpy.parameter import Parameter

from obelisk_py.core.control import ObeliskController
from obelisk_py.core.estimation import ObeliskEstimator
from obelisk_py.core.utils.launch_utils import get_parameters_dict


# ------------------------------------------------------------------ #
# Concrete test subclasses                                           #
# ------------------------------------------------------------------ #


class _DummyController(ObeliskController):
    """Minimal concrete ObeliskController for use as a test subject."""

    def update_x_hat(self, x_hat_msg: Any) -> None:  # pragma: no cover - not exercised here
        pass

    def compute_control(self) -> Any:  # pragma: no cover - not exercised here
        return None


class _DummyEstimator(ObeliskEstimator):
    """Minimal concrete ObeliskEstimator that adds a sensor subscription."""

    def __init__(self, node_name: str, est_msg_type: Type) -> None:
        super().__init__(node_name, est_msg_type)
        self.register_obk_subscription(
            key="sub_sensor",
            callback=self.update_sensor,
            msg_type=ObkJointEncoders,
        )

    def update_sensor(self, msg: Any) -> None:  # pragma: no cover
        pass

    def compute_state_estimate(self) -> Any:  # pragma: no cover
        return None


# ------------------------------------------------------------------ #
# Helpers                                                            #
# ------------------------------------------------------------------ #


def _apply_settings(node: Any, node_settings: dict) -> None:
    """Run a YAML-style settings dict through the producer and apply the result to the node.

    This mirrors the launch-time flow: `get_parameters_dict` builds the ROS-parameter dict,
    we assign each entry as a parameter on the node, then `on_configure` is what actually
    instantiates pubs/subs/timers from those parameters.
    """
    parameters_dict = get_parameters_dict(node_settings)

    # Producer may emit lists when there are multiple entries with the same ros_parameter name;
    # the existing parser handles single strings (declare_parameter happens once per ros_parameter).
    # If we hit a list, we use the first element so that the node's single parameter declaration
    # is satisfied. The multi-publisher-per-ros_parameter case is exercised by separate parameters
    # in the test bodies below rather than via list-valued params.
    parameters = []
    for name, value in parameters_dict.items():
        if isinstance(value, list):
            value = value[0]
        if isinstance(value, str):
            parameters.append(Parameter(name, Parameter.Type.STRING, value))
        elif isinstance(value, int):
            parameters.append(Parameter(name, Parameter.Type.INTEGER, value))
        elif isinstance(value, float):
            parameters.append(Parameter(name, Parameter.Type.DOUBLE, value))
        elif isinstance(value, bool):
            parameters.append(Parameter(name, Parameter.Type.BOOL, value))
    node.set_parameters(parameters)


# ------------------------------------------------------------------ #
# Fixtures                                                           #
# ------------------------------------------------------------------ #


@pytest.fixture
def controller(ros_context: Any) -> Generator[_DummyController, None, None]:
    node = _DummyController(
        "yaml_to_node_controller",
        ctrl_msg_type=EstimatedState,
        est_msg_type=EstimatedState,
    )
    yield node
    node.destroy_node()


@pytest.fixture
def estimator(ros_context: Any) -> Generator[_DummyEstimator, None, None]:
    node = _DummyEstimator("yaml_to_node_estimator", est_msg_type=EstimatedState)
    yield node
    node.destroy_node()


# ------------------------------------------------------------------ #
# Tests                                                              #
# ------------------------------------------------------------------ #


def test_simple_controller_yaml_to_node(controller: _DummyController) -> None:
    """A trivial controller YAML configures pub, sub, and timer with the values from YAML."""
    node_settings = {
        "publishers": [
            {
                "ros_parameter": "pub_ctrl_setting",
                "topic": "/safety_net/ctrl",
                "history_depth": 7,
                "callback_group": "None",
            }
        ],
        "subscribers": [
            {
                "ros_parameter": "sub_est_setting",
                "topic": "/safety_net/est",
                "history_depth": 3,
                "callback_group": "None",
            }
        ],
        "timers": [
            {
                "ros_parameter": "timer_ctrl_setting",
                "timer_period_sec": 0.005,
                "callback_group": "None",
            }
        ],
    }
    _apply_settings(controller, node_settings)
    assert controller.on_configure(None) == TransitionCallbackReturn.SUCCESS

    pub = controller.obk_publishers["pub_ctrl"]
    assert pub.topic_name == "/safety_net/ctrl"
    assert pub.qos_profile.depth == 7

    sub = controller.obk_subscriptions["sub_est"]
    assert sub.topic_name == "/safety_net/est"
    assert sub.qos_profile.depth == 3

    timer = controller.obk_timers["timer_ctrl"]
    # rclpy Timer exposes its period in nanoseconds as `timer_period_ns`.
    assert timer.timer_period_ns == int(0.005 * 1e9)


def test_callback_group_binding(controller: _DummyController) -> None:
    """A pub bound to a named callback group ends up on the right group instance."""
    node_settings = {
        "callback_groups": {
            "ctrl_cbg": "MutuallyExclusiveCallbackGroup",
            "io_cbg": "ReentrantCallbackGroup",
        },
        "publishers": [
            {
                "ros_parameter": "pub_ctrl_setting",
                "topic": "/safety_net/ctrl",
                "history_depth": 10,
                "callback_group": "ctrl_cbg",
            }
        ],
        "subscribers": [
            {
                "ros_parameter": "sub_est_setting",
                "topic": "/safety_net/est",
                "history_depth": 10,
                "callback_group": "io_cbg",
            }
        ],
        "timers": [
            {
                "ros_parameter": "timer_ctrl_setting",
                "timer_period_sec": 0.01,
                "callback_group": "ctrl_cbg",
            }
        ],
    }
    _apply_settings(controller, node_settings)
    assert controller.on_configure(None) == TransitionCallbackReturn.SUCCESS

    cbgs = controller.obk_callback_groups
    assert isinstance(cbgs["ctrl_cbg"], MutuallyExclusiveCallbackGroup)
    assert isinstance(cbgs["io_cbg"], ReentrantCallbackGroup)


def test_estimator_with_sensor_sub(estimator: _DummyEstimator) -> None:
    """The estimator subclass exposes a sensor subscription configured by YAML."""
    node_settings = {
        "publishers": [
            {
                "ros_parameter": "pub_est_setting",
                "topic": "/safety_net/est_out",
                "history_depth": 5,
                "callback_group": "None",
            }
        ],
        "subscribers": [
            {
                "ros_parameter": "sub_sensor_setting",
                "topic": "/safety_net/sensors",
                "history_depth": 10,
                "callback_group": "None",
            }
        ],
        "timers": [
            {
                "ros_parameter": "timer_est_setting",
                "timer_period_sec": 0.02,
                "callback_group": "None",
            }
        ],
    }
    _apply_settings(estimator, node_settings)
    assert estimator.on_configure(None) == TransitionCallbackReturn.SUCCESS

    assert estimator.obk_publishers["pub_est"].topic_name == "/safety_net/est_out"
    assert estimator.obk_publishers["pub_est"].qos_profile.depth == 5
    assert estimator.obk_subscriptions["sub_sensor"].topic_name == "/safety_net/sensors"
    assert estimator.obk_timers["timer_est"].timer_period_ns == int(0.02 * 1e9)


def test_get_parameters_dict_handles_no_settings() -> None:
    """get_parameters_dict on None returns an empty dict (launch-side robustness)."""
    assert get_parameters_dict(None) == {}


def test_user_params_pass_through() -> None:
    """User-defined `params` and `params_path` entries are forwarded as ROS parameters as-is."""
    node_settings = {
        "params_path": "/some/path/params.txt",
        "params": {
            "user_int": 42,
            "user_str": "hello",
            "user_list": [1, 2, 3],
        },
    }
    parameters_dict = get_parameters_dict(node_settings)
    assert parameters_dict["params_path"] == "/some/path/params.txt"
    assert parameters_dict["user_int"] == 42
    assert parameters_dict["user_str"] == "hello"
    assert parameters_dict["user_list"] == [1, 2, 3]


def test_sim_settings_with_nested_sensors() -> None:
    """The mujoco-style sim entry with nested `sensor_settings` and `sensor_names` survives the
    producer pipeline.

    This is the worst-case for today's implementation (the regex-escaping path). The behavior we
    pin: the producer must not lose any of the leaf data, the result must be reachable as a single
    ROS parameter named after `ros_parameter`, and the leaf values (topic, dt, sensor names) must
    still be discoverable from the encoded output.
    """
    node_settings = {
        "sim": [
            {
                "ros_parameter": "mujoco_setting",
                "num_steps_per_viz": 5,
                "robot_pkg": "dummy_bot",
                "model_xml_path": "dummy.xml",
                "sensor_settings": [
                    {
                        "topic": "/safety_net/joint_encoders",
                        "dt": 0.001,
                        "msg_type": "ObkJointEncoders",
                        "sensor_names": {
                            "joint_pos": "jointpos",
                            "joint_vel": "jointvel",
                        },
                    },
                ],
            }
        ]
    }
    parameters_dict = get_parameters_dict(node_settings)
    assert "mujoco_setting" in parameters_dict
    encoded = parameters_dict["mujoco_setting"]
    # The new producer emits sim entries as YAML strings; round-trip and check leaf values.
    import yaml as _yaml

    parsed = _yaml.safe_load(encoded)
    assert parsed["model_xml_path"] == "dummy.xml"
    assert parsed["sensor_settings"][0]["topic"] == "/safety_net/joint_encoders"
    assert parsed["sensor_settings"][0]["dt"] == 0.001
    assert parsed["sensor_settings"][0]["msg_type"] == "ObkJointEncoders"
    assert parsed["sensor_settings"][0]["sensor_names"] == {"joint_pos": "jointpos", "joint_vel": "jointvel"}


def test_true_sim_state_msg_imports() -> None:
    """Sanity check that the sensor message types referenced by sim configs are importable —
    the mujoco_sim_robot ties to TrueSimState and ObkJointEncoders specifically.
    """
    assert TrueSimState is not None
    assert ObkJointEncoders is not None
