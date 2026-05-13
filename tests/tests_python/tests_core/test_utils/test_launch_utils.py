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


_ASSETS = Path(__file__).parent.parent.parent / "test_assets"


def test_load_config_file(test_config: Dict[str, Any]) -> None:
    """load_config_file reads YAML files (used by the launch entry point)."""
    result = load_config_file(_ASSETS / "test_config.yaml")
    assert result == test_config

    with pytest.raises(FileNotFoundError):
        load_config_file("non_existent.yaml")


# ---------------------------------------------------------------------- #
# include: directive — composition tests                                 #
# ---------------------------------------------------------------------- #


def test_load_config_no_include_returns_same_dict(test_config: Dict[str, Any]) -> None:
    """A YAML without an `include:` key behaves exactly as before (regression check)."""
    result = load_config_file(_ASSETS / "test_config.yaml")
    assert "include" not in result
    assert result == test_config


def test_load_config_with_include_merges_and_strips_directive() -> None:
    """compose_a includes compose_b which includes compose_c. The include key is dropped from
    the result, scalars take the outermost value, and list-shaped sections concat in include order
    (innermost first).
    """
    result = load_config_file(_ASSETS / "compose_a.yaml")

    assert "include" not in result
    # config: outermost (a) wins
    assert result["config"] == "composed_a"
    # control: c, b, a appended in that order (deepest include first, outermost last)
    ctrl_pkgs = [entry["pkg"] for entry in result["control"]]
    assert ctrl_pkgs == ["c_pkg", "b_pkg", "a_pkg"]
    # estimation only defined in c, propagates up unchanged
    assert result["estimation"][0]["pkg"] == "c_est_pkg"


def test_load_config_dict_section_deep_merge() -> None:
    """joystick is dict-shaped: deep-merge with parent winning on per-key conflicts. b sets `on`
    and `sub_topic`; a overrides `pub_topic`.
    """
    result = load_config_file(_ASSETS / "compose_a.yaml")
    assert result["joystick"] == {
        "on": True,
        "pub_topic": "/from_a",  # a overrode b's value
        "sub_topic": "/from_b_sub",  # only b set this; survives
    }


def test_load_config_recursive_includes_three_deep() -> None:
    """A includes B, B includes C; C's contributions appear in the final result."""
    result = load_config_file(_ASSETS / "compose_a.yaml")
    # only c defines an estimation list, and it has one entry
    assert len(result["estimation"]) == 1
    assert result["estimation"][0]["executable"] == "c_est"


def test_load_config_cycle_detection_raises() -> None:
    """An include cycle raises RuntimeError mentioning the chain — never hangs."""
    with pytest.raises(RuntimeError, match="Cycle"):
        load_config_file(_ASSETS / "compose_cycle_a.yaml")


def test_load_config_relative_includes_resolve_to_parent_dir(tmp_path: Path) -> None:
    """Relative paths in `include:` resolve against the parent file's directory, not the
    obelisk_ros package share dir.
    """
    # Build a small two-file tree in a fresh tmpdir so we know the dir is unrelated to share/.
    sub = tmp_path / "configs" / "subdir"
    sub.mkdir(parents=True)
    (sub / "leaf.yaml").write_text("config: leaf\ncontrol:\n  - pkg: leaf_pkg\n    executable: e\n")
    (sub / "root.yaml").write_text(
        "config: root\ninclude:\n  - leaf.yaml\ncontrol:\n  - pkg: root_pkg\n    executable: e\n"
    )

    result = load_config_file(sub / "root.yaml")
    assert [c["pkg"] for c in result["control"]] == ["leaf_pkg", "root_pkg"]
    assert result["config"] == "root"


def test_load_config_absolute_include_path_works(tmp_path: Path) -> None:
    """An absolute path inside `include:` is used as-is."""
    leaf = tmp_path / "leaf.yaml"
    leaf.write_text("control:\n  - pkg: abs_pkg\n    executable: e\n")
    root = tmp_path / "root.yaml"
    root.write_text(f"include:\n  - {leaf}\n")

    result = load_config_file(root)
    assert result["control"][0]["pkg"] == "abs_pkg"


def test_load_config_dummy_composed_resolves_to_full_stack() -> None:
    """End-to-end smoke test: dummy_composed.yaml (which uses `include:`) loads into a dict that
    looks like a single-file config — control + estimation + robot + joystick all present, no
    leftover `include:` key."""
    composed = load_config_file("dummy_composed.yaml")
    assert "include" not in composed
    assert composed["config"] == "dummy_composed"
    assert len(composed["control"]) == 1
    assert composed["control"][0]["pkg"] == "obelisk_control_cpp"
    assert len(composed["estimation"]) == 1
    assert len(composed["robot"]) == 1
    assert composed["robot"][0]["is_simulated"] is True
    assert composed["joystick"]["on"] is True


def test_get_parameters_dict_bundles_obelisk_sections(test_config: Dict[str, Any]) -> None:
    """A controller node's settings collapse into a single obelisk_settings YAML string."""
    node_settings = test_config["control"][0]
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
    node_settings = test_config["control"]
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


# ---------------------------------------------------------------------- #
# generic `nodes:` section + optional-section coverage                   #
# ---------------------------------------------------------------------- #


def test_get_launch_actions_accepts_node_type(test_global_state_node: LifecycleNode) -> None:
    """The generic ``nodes:`` section uses ``node_type='node'`` and goes through the same launch
    pipeline as control/estimation/robot/sensing.
    """
    settings = [{"pkg": "p", "executable": "e"}]
    launch_actions = get_launch_actions_from_node_settings(settings, "node", test_global_state_node)
    assert len(launch_actions) == 8  # 1 LifecycleNode + 7 lifecycle handlers, same as the others
    assert isinstance(launch_actions[0], LifecycleNode)


def test_merge_concats_nodes_section() -> None:
    """The new ``nodes:`` section is treated as list-shaped and concatenates across includes."""
    base = {"nodes": [{"pkg": "a", "executable": "x"}]}
    override = {"nodes": [{"pkg": "b", "executable": "y"}]}
    from obelisk_py.core.utils.launch_utils import _merge_obelisk_configs  # private but stable

    merged = _merge_obelisk_configs(base, override)
    assert [n["pkg"] for n in merged["nodes"]] == ["a", "b"]


def test_load_dummy_minimal_yaml() -> None:
    """End-to-end smoke test: dummy_minimal.yaml has only a ``nodes:`` section — no control,
    estimation, robot, or sensing — and the loader returns it cleanly.
    """
    cfg = load_config_file("dummy_minimal.yaml")
    assert cfg["config"] == "dummy_minimal"
    assert "control" not in cfg
    assert "estimation" not in cfg
    assert "robot" not in cfg
    assert "sensing" not in cfg
    assert len(cfg["nodes"]) == 1
    assert cfg["nodes"][0]["executable"] == "heartbeat_node"
