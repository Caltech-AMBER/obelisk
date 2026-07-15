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


# ---------------------------------------------------------------------- #
# ${VAR} environment-variable expansion                                   #
# ---------------------------------------------------------------------- #


def test_load_config_expands_env_vars_in_values(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """``${VAR}`` in string values (node params and sim config_path) is expanded at load time."""
    monkeypatch.setenv("TEST_ROOT", "/opt/deploy")
    cfg = tmp_path / "cfg.yaml"
    cfg.write_text(
        "control:\n"
        "  - pkg: p\n"
        "    executable: e\n"
        "    params:\n"
        "      seq_path: ${TEST_ROOT}/graph/seqs\n"
        "    sim:\n"
        "      - ros_parameter: mujoco_setting\n"
        "        sensor_settings:\n"
        "          - config_path: ${TEST_ROOT}/scan/depth.yml\n"
    )
    result = load_config_file(cfg)
    entry = result["control"][0]
    assert entry["params"]["seq_path"] == "/opt/deploy/graph/seqs"
    assert entry["sim"][0]["sensor_settings"][0]["config_path"] == "/opt/deploy/scan/depth.yml"


def test_load_config_env_var_undefined_raises(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """An undefined ``${VAR}`` raises KeyError naming both the variable and the offending file."""
    monkeypatch.delenv("DEFINITELY_UNSET_VAR", raising=False)
    cfg = tmp_path / "cfg.yaml"
    cfg.write_text("control:\n  - pkg: p\n    executable: e\n    params:\n      x: ${DEFINITELY_UNSET_VAR}/y\n")
    with pytest.raises(KeyError, match="DEFINITELY_UNSET_VAR"):
        load_config_file(cfg)


def test_load_config_env_var_leaves_scalars_and_bare_dollar(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Non-string scalars pass through untouched and a bare ``$`` (no braces) is never expanded."""
    monkeypatch.setenv("TEST_ROOT", "/opt/deploy")
    cfg = tmp_path / "cfg.yaml"
    cfg.write_text(
        "control:\n"
        "  - pkg: p\n"
        "    executable: e\n"
        "    params:\n"
        "      seq_nodes: [5.0, 0.0, 0.0]\n"
        "      cyclic: false\n"
        "      note: 'costs $5'\n"
    )
    params = load_config_file(cfg)["control"][0]["params"]
    assert params["seq_nodes"] == [5.0, 0.0, 0.0]
    assert params["cyclic"] is False
    assert params["note"] == "costs $5"


def test_load_config_env_var_in_include_path(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """``${VAR}`` inside an ``include:`` path is expanded before the include is resolved."""
    leaf = tmp_path / "leaf.yaml"
    leaf.write_text("control:\n  - pkg: leaf_pkg\n    executable: e\n")
    monkeypatch.setenv("TEST_INC_DIR", str(tmp_path))
    root = tmp_path / "root.yaml"
    root.write_text("include:\n  - ${TEST_INC_DIR}/leaf.yaml\n")
    result = load_config_file(root)
    assert result["control"][0]["pkg"] == "leaf_pkg"


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


# ---------------------------------------------------------------------- #
# include: directive — keyed list-merge + anonymous-entry freezing        #
# ---------------------------------------------------------------------- #


def _compose(tmp_path: Path, base_yaml: str, ext_yaml: str) -> Dict[str, Any]:
    """Write base.yaml + ext.yaml (ext includes base) in tmp_path and load the extension."""
    (tmp_path / "base.yaml").write_text(base_yaml)
    (tmp_path / "ext.yaml").write_text("include:\n  - base.yaml\n" + ext_yaml)
    return load_config_file(tmp_path / "ext.yaml")


def test_keyed_merge_nested_override_by_name_and_ros_parameter(tmp_path: Path) -> None:
    """A restated robot entry (by name) overrides one nested sim field, inheriting the rest."""
    result = _compose(
        tmp_path,
        base_yaml=(
            "robot:\n"
            "  - name: g1_sim\n"
            "    pkg: unitree\n"
            "    executable: sim\n"
            "    sim:\n"
            "      - ros_parameter: mujoco_setting\n"
            "        model_xml_path: A\n"
            "        sensor_settings:\n"
            "          - topic: /enc\n"
            "            dt: 0.002\n"
        ),
        ext_yaml=(
            "robot:\n"
            "  - name: g1_sim\n"
            "    sim:\n"
            "      - ros_parameter: mujoco_setting\n"
            "        model_xml_path: B\n"
        ),
    )
    assert len(result["robot"]) == 1  # restated entry overrode, did not duplicate
    sim = result["robot"][0]["sim"]
    assert len(sim) == 1
    assert sim[0]["model_xml_path"] == "B"  # the only changed field
    # everything not restated is inherited verbatim from the base
    assert sim[0]["sensor_settings"] == [{"topic": "/enc", "dt": 0.002}]


def test_keyed_merge_scalar_list_replaces(tmp_path: Path) -> None:
    """A list of scalars is replaced wholesale, not keyed-merged."""
    result = _compose(
        tmp_path,
        base_yaml=(
            "robot:\n"
            "  - name: r\n"
            "    pkg: p\n"
            "    executable: e\n"
            "    sim:\n"
            "      - ros_parameter: mujoco_setting\n"
            "        geom_groups: [0, 1, 2]\n"
        ),
        ext_yaml=(
            "robot:\n"
            "  - name: r\n"
            "    sim:\n"
            "      - ros_parameter: mujoco_setting\n"
            "        geom_groups: [2]\n"
        ),
    )
    assert result["robot"][0]["sim"][0]["geom_groups"] == [2]


def test_keyed_merge_key_ros_parameter_normalization(tmp_path: Path) -> None:
    """`key: pub_ctrl` and `ros_parameter: pub_ctrl_setting` identify the same publisher."""
    result = _compose(
        tmp_path,
        base_yaml=(
            "control:\n"
            "  - name: c\n"
            "    pkg: p\n"
            "    executable: e\n"
            "    publishers:\n"
            "      - ros_parameter: pub_ctrl_setting\n"
            "        topic: /a\n"
        ),
        ext_yaml=(
            "control:\n"
            "  - name: c\n"
            "    publishers:\n"
            "      - key: pub_ctrl\n"
            "        topic: /b\n"
        ),
    )
    pubs = result["control"][0]["publishers"]
    assert len(pubs) == 1  # matched, not appended
    assert pubs[0]["topic"] == "/b"


def test_keyless_entries_append_and_get_unknown_labels(tmp_path: Path) -> None:
    """Keyless node entries stack (today's concat) and receive reserved UNKNOWN# names."""
    result = _compose(
        tmp_path,
        base_yaml="control:\n  - pkg: a\n    executable: x\n",
        ext_yaml="control:\n  - pkg: b\n    executable: y\n",
    )
    assert [c["pkg"] for c in result["control"]] == ["a", "b"]  # base first, append
    assert [c["name"] for c in result["control"]] == ["UNKNOWN0", "UNKNOWN1"]


def test_keyed_merge_topic_keyed_sensor_merge_and_append(tmp_path: Path) -> None:
    """sensor_settings merge by `topic`: matched entry updates, new topic appends."""
    result = _compose(
        tmp_path,
        base_yaml=(
            "robot:\n"
            "  - name: r\n"
            "    pkg: p\n"
            "    executable: e\n"
            "    sim:\n"
            "      - ros_parameter: mujoco_setting\n"
            "        sensor_settings:\n"
            "          - topic: /depth\n"
            "            dt: 0.033\n"
        ),
        ext_yaml=(
            "robot:\n"
            "  - name: r\n"
            "    sim:\n"
            "      - ros_parameter: mujoco_setting\n"
            "        sensor_settings:\n"
            "          - topic: /depth\n"
            "            dt: 0.05\n"
            "          - topic: /new\n"
            "            dt: 0.01\n"
        ),
    )
    sensors = result["robot"][0]["sim"][0]["sensor_settings"]
    assert len(sensors) == 2
    by_topic = {s["topic"]: s for s in sensors}
    assert by_topic["/depth"]["dt"] == 0.05  # matched and updated
    assert "/new" in by_topic  # appended


def test_name_on_override_does_not_match_keyless_base(tmp_path: Path) -> None:
    """The key must be on the BASE entry to override it; a name only on the override appends."""
    result = _compose(
        tmp_path,
        base_yaml="control:\n  - pkg: a\n    executable: x\n",
        ext_yaml="control:\n  - name: foo\n    pkg: a\n    executable: x\n",
    )
    assert len(result["control"]) == 2
    # keyless base entry is frozen and labeled; override keeps its real name
    assert result["control"][0]["name"] == "UNKNOWN0"
    assert result["control"][1]["name"] == "foo"


def test_unknown_placeholder_cannot_hijack_frozen_entry(tmp_path: Path) -> None:
    """A downstream `name: UNKNOWN0` cannot address a frozen (anonymous) base entry."""
    result = _compose(
        tmp_path,
        base_yaml="control:\n  - pkg: a\n    executable: x\n",
        ext_yaml="control:\n  - name: UNKNOWN0\n    pkg: b\n    executable: y\n",
    )
    # the placeholder does NOT overwrite the base entry; it appends
    assert [c["pkg"] for c in result["control"]] == ["a", "b"]
    # both are anonymous, so both get fresh dense labels (the override is renumbered)
    assert [c["name"] for c in result["control"]] == ["UNKNOWN0", "UNKNOWN1"]


def test_standalone_config_is_not_labeled(tmp_path: Path) -> None:
    """A config with no `include:` never triggers labeling — returned exactly as written."""
    standalone = tmp_path / "standalone.yaml"
    standalone.write_text("control:\n  - pkg: a\n    executable: x\n")
    result = load_config_file(standalone)
    assert "name" not in result["control"][0]


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


def test_get_launch_actions_node_names(test_global_state_node: LifecycleNode) -> None:
    """A real config `name` names the ROS node; anonymous/placeholder entries keep the positional fallback."""
    node_settings = [
        {"pkg": "p", "executable": "e", "name": "graph_planner"},
        {"pkg": "p", "executable": "e"},
        {"pkg": "p", "executable": "e", "name": "UNKNOWN0"},
    ]
    launch_actions = get_launch_actions_from_node_settings(node_settings, "control", test_global_state_node)
    nodes = [a for a in launch_actions if isinstance(a, LifecycleNode)]
    # the raw constructor name (node_name is only readable after the action executes)
    names = [n._Node__node_name for n in nodes]
    assert names == ["obelisk_control_graph_planner", "obelisk_control_1", "obelisk_control_2"]


def test_get_launch_actions_from_node_settings_invalid_type(test_global_state_node: LifecycleNode) -> None:
    """An unknown node type asserts."""
    invalid = {
        "pkg": "p",
        "executable": "e",
        "callback_groups": {"cbg1": "MutuallyExclusiveCallbackGroup"},
    }
    with pytest.raises(AssertionError):
        get_launch_actions_from_node_settings(invalid, "invalid_type", test_global_state_node)
