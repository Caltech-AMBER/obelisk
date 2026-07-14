import os
import re
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Union

import launch
import launch_ros
import lifecycle_msgs.msg
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import EmitEvent, OpaqueFunction, RegisterEventHandler
from launch_ros.actions import LifecycleNode, Node
from launch_ros.events.lifecycle import ChangeState
from ruamel.yaml import YAML

# Sections of a node's settings dict that Obelisk owns and bundles into the single
# `obelisk_settings` ROS parameter at launch time.
_OBELISK_SETTINGS_KEYS = ("publishers", "subscribers", "timers", "sim", "callback_groups")


def load_config_file(file_path: Union[str, Path], package_name: Optional[str] = None) -> Dict:
    """Loads an Obelisk configuration file, recursively expanding ``include:`` directives.

    The file at ``file_path`` is the *primary* config: an absolute path is used as-is, otherwise
    the path resolves against the ``config/`` subdirectory of an installed ROS2 package (default
    ``obelisk_ros``).

    If the loaded YAML has a top-level ``include:`` key (a list of paths), each entry is loaded
    recursively and merged into a single dict before the parent's own keys are applied. Paths inside
    an ``include:`` resolve **relative to the directory of the file containing the include**;
    absolute paths still pass through unchanged. The ``include:`` key is stripped from the result.

    Merge rules (parent = file containing the include, children = files it includes) are one
    uniform recursive merge with the parent winning:
      * Dicts deep-merge key by key (``joystick``, ``viz``, and any nested mapping).
      * Lists of mappings **keyed-merge**: an entry whose identity (first of ``name`` / ``id`` /
        ``key`` / ``ros_parameter`` / ``topic``) matches an existing entry is merged onto it;
        entries with no match — or no identity — append. Lists of scalars are replaced.
      * Scalar / other keys (e.g. ``config``, ``params_path``): parent wins.

    After the merge, identity-less entries in the node-level list sections (and ``viz.viz_nodes``)
    are given a reserved ``name: UNKNOWN<n>``; this placeholder is never used as an identity, so an
    entry must carry a real ``name``/key to be overridable by a downstream include.

    Parameters:
        file_path: path to the configuration file.
        package_name: the ROS2 package whose ``config/`` dir to resolve relative ``file_path``
            against. If None, ``obelisk_ros`` is used.

    Returns:
        config: A dictionary containing the merged configuration settings.

    Raises:
        FileNotFoundError: if the file (or any include) cannot be located/parsed.
        RuntimeError: if the include graph contains a cycle.
    """
    abs_file_path = _resolve_primary_config_path(file_path, package_name)
    return _load_config_recursive(abs_file_path, stack=[])


# Matches ``${VAR}`` references in config string values. Only the braced form is
# expanded (never bare ``$VAR``) so an incidental ``$`` in a value is left untouched.
_ENV_VAR_RE = re.compile(r"\$\{([A-Za-z_][A-Za-z0-9_]*)\}")


def _expand_env_vars(node: Any, source: Path) -> Any:
    """Recursively expand ``${VAR}`` references in every string value of a loaded config.

    Runs on each YAML file right after it is loaded (before ``include:`` processing and merging),
    so env vars work in ``include:`` paths, node ``params``, ``sim`` sensor ``config_path`` entries,
    and any other string leaf. Expansion happens here, in the launch process, before values are
    serialized into ROS parameters, so downstream nodes (Python *and* C++) receive fully-resolved
    absolute paths and need no env-var awareness of their own.

    Only the braced ``${VAR}`` form is expanded; a bare ``$`` is left alone. An **undefined**
    variable raises ``KeyError`` (naming the offending file) so a typo fails fast at launch rather
    than surfacing later as a confusing file-not-found deep inside a node. Non-string scalars pass
    through untouched; dict keys are never expanded, only values.

    Parameters:
        node: the loaded config value (dict / list / str / scalar) to expand in place-ish.
        source: the file the value came from, used only for error messages.

    Returns:
        The value with all ``${VAR}`` references in string leaves expanded.

    Raises:
        KeyError: if a referenced environment variable is not set.
    """
    if isinstance(node, dict):
        return {k: _expand_env_vars(v, source) for k, v in node.items()}
    if isinstance(node, list):
        return [_expand_env_vars(v, source) for v in node]
    if isinstance(node, str):

        def _sub(match: "re.Match") -> str:
            name = match.group(1)
            if name not in os.environ:
                raise KeyError(f"Undefined environment variable ${{{name}}} referenced in obelisk config {source}")
            return os.environ[name]

        return _ENV_VAR_RE.sub(_sub, node)
    return node


def _resolve_primary_config_path(file_path: Union[str, Path], package_name: Optional[str]) -> Path:
    """Resolve the primary (top-level) config path: absolute as-is, else under <pkg>/share/config/."""
    file_path = str(file_path)
    if not file_path.startswith("/"):
        try:
            obk_ros_dir = get_package_share_directory("obelisk_ros" if package_name is None else package_name)
        except Exception as e:
            raise FileNotFoundError(
                f"Could not find package {package_name}. If you supply a relative path to the config file, it must lie "
                "in a subdirectory named 'config' of either the obelisk_ros package or some other ROS2 package of your "
                "choice. Otherwise, you can supply an absolute path to the config file."
            ) from e
        return Path(obk_ros_dir) / "config" / file_path
    return Path(file_path)


def _load_config_recursive(abs_file_path: Path, stack: List[Path]) -> Dict:
    """Load a single YAML and recursively expand any ``include:`` it has, merging the results.

    ``stack`` tracks the chain of resolved-absolute paths so we can detect include cycles.
    """
    abs_file_path = abs_file_path.resolve()
    if abs_file_path in stack:
        chain = " -> ".join(str(p) for p in stack + [abs_file_path])
        raise RuntimeError(f"Cycle detected in obelisk config include chain: {chain}")

    loader = YAML(typ="safe")
    try:
        raw = loader.load(abs_file_path)
    except Exception as e:
        raise FileNotFoundError(f"Could not load a configuration file at {abs_file_path}!") from e

    # An empty or null YAML file should be treated as an empty mapping rather than failing.
    if raw is None:
        raw = {}

    # Expand ${VAR} references before include-processing and merging, so env vars work in
    # include paths and in every string leaf (params, sim config_path, etc.). Done per-file
    # here (not once on the merged result) so includes are expanded in their own file's right.
    raw = _expand_env_vars(raw, abs_file_path)

    includes = raw.pop("include", []) or []
    if not isinstance(includes, list):
        raise ValueError(
            f"`include:` in {abs_file_path} must be a list of YAML paths; got {type(includes).__name__}."
        )

    new_stack = stack + [abs_file_path]
    base_dir = abs_file_path.parent

    merged: Dict[str, Any] = {}
    for inc in includes:
        inc_path = Path(str(inc))
        # Includes are resolved relative to the directory of the file containing the include.
        # Absolute paths pass through unchanged.
        if not inc_path.is_absolute():
            inc_path = base_dir / inc_path
        included = _load_config_recursive(inc_path, new_stack)
        merged = _merge_config(merged, included)

    # Parent's own keys take precedence over included ones.
    result = _merge_config(merged, raw)
    # Only the outermost call (stack == []) that actually composed (had includes) labels
    # anonymous node entries; a standalone, non-composed config is returned unchanged.
    if not stack and includes:
        _assign_anonymous_ids(result)
    return result


# Node-level list sections: keyed-merged like everything else, and the only sections the
# anonymous-id labeling pass writes a synthetic ``name`` into.
_LIST_SECTIONS = ("control", "estimation", "robot", "sensing")

# Identity precedence for keyed list-merge. First field present on a list element
# identifies it; ``ros_parameter`` is normalized into the ``key`` namespace.
_KEY_PRECEDENCE = ("name", "id", "key", "ros_parameter", "topic")
_SETTING_SUFFIX = "_setting"

# Reserved placeholder for entries that declare no key. Assigned post-merge as a stable,
# visible handle; never treated as a real identity, so it can never be overwritten by a
# later merge (an entry must have a real name/key to be overridable downstream).
_ANON_PREFIX = "UNKNOWN"
_ANON_RE = re.compile(rf"^{_ANON_PREFIX}\d+$")


def _entry_identity(entry: Any) -> Optional[tuple]:
    """Identity of a list element for keyed merging, or None if it has no real key.

    Precedence: name -> id -> key -> ros_parameter -> topic. ``ros_parameter`` is
    normalized into the ``key`` namespace (trailing ``_setting`` stripped, mirroring
    ``_strip_ros_parameter``), so ``ros_parameter: pub_ctrl_setting`` and ``key: pub_ctrl``
    match. A field whose value is the reserved ``UNKNOWN<n>`` placeholder is skipped, so
    anonymous entries stay anonymous (and unaddressable) even across re-merges. Returns a
    ``(namespace, value)`` tuple so values from different fields never collide; None means
    "no identity" -> append-only, never a merge target.
    """
    if not isinstance(entry, dict):
        return None
    for field in _KEY_PRECEDENCE:
        if field in entry:
            val = entry[field]
            if isinstance(val, str) and _ANON_RE.match(val):
                continue  # reserved placeholder is never a match key
            if field == "ros_parameter":
                if isinstance(val, str) and val.endswith(_SETTING_SUFFIX):
                    val = val[: -len(_SETTING_SUFFIX)]
                return ("key", val)
            return (field, val)
    return None


def _merge_config(base: Any, override: Any) -> Any:
    """Recursively merge ``override`` onto ``base`` (override wins).

    dict+dict  -> merge key by key (recurse on shared keys)
    list+list  -> keyed merge when both are non-empty lists of dicts (see
                  ``_merge_lists``); otherwise ``override`` replaces (scalar vectors)
    else       -> ``override`` replaces ``base``
    """
    if isinstance(base, dict) and isinstance(override, dict):
        out: Dict[str, Any] = dict(base)
        for k, v in override.items():
            out[k] = _merge_config(out[k], v) if k in out else v
        return out
    if isinstance(base, list) and isinstance(override, list):
        return _merge_lists(base, override)
    return override


def _merge_lists(base: List, override: List) -> List:
    """Keyed merge of two lists; see ``_merge_config`` for when it applies.

    Both lists non-empty and all-dict -> merge by identity: matched override elements
    deep-merge onto the base element (preserving base order); unmatched or keyless override
    elements append. Otherwise ``override`` replaces (scalar vectors, empty, or mixed).
    """
    if not (
        base
        and override
        and all(isinstance(e, dict) for e in base)
        and all(isinstance(e, dict) for e in override)
    ):
        return list(override)

    out = [dict(e) for e in base]
    index: Dict[tuple, int] = {}
    for i, e in enumerate(out):
        ident = _entry_identity(e)
        if ident is not None:
            index.setdefault(ident, i)  # first occurrence wins if base has dups
    for e in override:
        ident = _entry_identity(e)
        if ident is not None and ident in index:
            i = index[ident]
            out[i] = _merge_config(out[i], e)
        else:
            out.append(e)
            if ident is not None:
                index.setdefault(ident, len(out) - 1)
    return out


def _assign_anonymous_ids(config: Dict) -> None:
    """Give every identity-less node entry a reserved, non-matchable ``name: UNKNOWN<n>``.

    Mutates ``config`` in place. Runs once on the fully-merged top-level config. Only the
    node-level list sections (``_LIST_SECTIONS``) and ``viz.viz_nodes`` are touched -- never
    ``key``/``ros_parameter``/``topic`` and never ``params``. Entries that already carry a
    real identity are left untouched; only entries with ``_entry_identity(...) is None`` get
    a placeholder. Numbering is per-list and dense (``UNKNOWN0``, ``UNKNOWN1``, ...). Because
    the placeholder is excluded from ``_entry_identity``, an anonymous entry can never be
    addressed by a downstream merge -- to override an entry you must give it a real ``name``
    (or other key) in the base.
    """
    targets: List[List] = [config[s] for s in _LIST_SECTIONS if isinstance(config.get(s), list)]
    viz = config.get("viz")
    if isinstance(viz, dict) and isinstance(viz.get("viz_nodes"), list):
        targets.append(viz["viz_nodes"])
    for entries in targets:
        n = 0
        for entry in entries:
            if isinstance(entry, dict) and _entry_identity(entry) is None:
                entry["name"] = f"{_ANON_PREFIX}{n}"
                n += 1


def _strip_ros_parameter(entry: Dict[str, Any]) -> Dict[str, Any]:
    """Return a shallow copy of an entry with ``ros_parameter`` dropped, deriving ``key`` if needed.

    The pre-refactor YAML schema named each pub/sub/timer entry by ``ros_parameter`` (e.g.
    ``pub_ctrl_setting``). The new schema uses ``key`` (e.g. ``pub_ctrl``). For YAMLs that haven't
    been migrated yet, derive ``key`` by stripping a trailing ``_setting``.
    """
    cleaned = {k: v for k, v in entry.items() if k != "ros_parameter"}
    if "key" not in cleaned and "ros_parameter" in entry:
        rp = entry["ros_parameter"]
        if isinstance(rp, str):
            cleaned["key"] = rp[: -len("_setting")] if rp.endswith("_setting") else rp
    return cleaned


def _build_obelisk_settings(node_settings: Dict) -> str:
    """Bundle the publishers / subscribers / timers / callback_groups sections into a single YAML string.

    Returns the empty string if the node has no Obelisk-internal sections (caller can then skip
    setting the ``obelisk_settings`` ROS parameter entirely if desired, though setting it to an
    empty string is also valid — the node treats empty as "no components").

    Note: ``sim`` entries are *not* bundled here. They are emitted as separate ROS parameters
    (one per ``ros_parameter`` in the entry, e.g. ``mujoco_setting``) so that simulator nodes can
    read their dedicated setting independently. Each sim entry's value is YAML-encoded.
    """
    bundle: Dict[str, Any] = {}
    for section in ("publishers", "subscribers", "timers"):
        if section in node_settings and node_settings[section] is not None:
            bundle[section] = [_strip_ros_parameter(entry) for entry in node_settings[section]]
    if "callback_groups" in node_settings and node_settings["callback_groups"] is not None:
        bundle["callback_groups"] = dict(node_settings["callback_groups"])
    if not bundle:
        return ""
    return yaml.safe_dump(bundle, default_flow_style=False, sort_keys=False)


def _build_sim_parameters(sim_entries: List[Dict[str, Any]]) -> Dict[str, str]:
    """Emit one YAML-string ROS parameter per sim entry, named by its ``ros_parameter`` field.

    Each entry's content (minus the ``ros_parameter`` key itself) is yaml.safe_dumped and used as
    the value of the ROS parameter. The simulator node parses the YAML in its on_configure.
    """
    out: Dict[str, str] = {}
    for entry in sim_entries:
        if "ros_parameter" not in entry:
            raise KeyError("Each sim entry must specify a `ros_parameter` for the simulator's settings parameter.")
        param_name = entry["ros_parameter"]
        body = {k: v for k, v in entry.items() if k != "ros_parameter"}
        out[param_name] = yaml.safe_dump(body, default_flow_style=False, sort_keys=False)
    return out


def get_parameters_dict(node_settings: Dict) -> Dict:
    """Returns the parameters dictionary corresponding to the settings of a node.

    The Obelisk-internal sections (``publishers``, ``subscribers``, ``timers``, ``sim``,
    ``callback_groups``) are bundled into a single ``obelisk_settings`` ROS parameter whose value is
    a YAML string. User-defined ``params`` and ``params_path`` entries flow through as separate ROS
    parameters, unchanged.

    Parameters:
        node_settings: the settings dictionary of an Obelisk node.

    Returns:
        A dictionary with the ROS parameter names as keys and the corresponding settings as values.
    """
    if node_settings is None:
        return {}

    parameters_dict: Dict[str, Any] = {}

    bundled = _build_obelisk_settings(node_settings)
    if bundled:
        parameters_dict["obelisk_settings"] = bundled

    if "sim" in node_settings and node_settings["sim"] is not None:
        parameters_dict.update(_build_sim_parameters(node_settings["sim"]))

    if "params_path" in node_settings:
        parameters_dict["params_path"] = node_settings["params_path"]
    if "params" in node_settings:
        parameters_dict.update(node_settings["params"])

    return parameters_dict


def get_launch_actions_from_node_settings(
    node_settings: Dict,
    node_type: str,
    global_state_node: LifecycleNode,
    auto_start: str = "false",
) -> List[LifecycleNode]:
    """Returns the launch actions associated with a node.

    Parameters:
        node_settings: the settings dictionary of an Obelisk node.
        node_type: the type of the node.
        global_state_node: the global state node. All Obelisk nodes' lifecycle states match this node's state.
        auto_start: the bringup's (lowercased) auto_start launch argument; see get_handlers.

    Returns:
        A list of launch actions.
    """
    assert node_type in ["control", "estimation", "robot", "sensing", "viz"]

    def _single_component_launch_actions(node_settings: Dict, suffix: Optional[int] = None) -> List:
        package = node_settings["pkg"]
        executable = node_settings["executable"]
        parameters_dict = get_parameters_dict(node_settings)
        launch_actions = []
        component_node = LifecycleNode(
            namespace="",
            name=f"obelisk_{node_type}" if suffix is None else f"obelisk_{node_type}_{suffix}",
            package=package,
            executable=executable,
            output="both",
            parameters=[parameters_dict],
        )
        launch_actions += [component_node]
        launch_actions += get_handlers(component_node, global_state_node, auto_start)
        return launch_actions

    node_launch_actions = []
    for i, node_setting in enumerate(node_settings):
        node_launch_actions += _single_component_launch_actions(node_setting, suffix=i)
    return node_launch_actions


def get_launch_actions_from_viz_settings(
    settings: Dict, global_state_node: LifecycleNode, auto_start: str = "false"
) -> List[LifecycleNode]:
    """Gets and configures all the launch actions related to viz given the settings from the yaml."""
    launch_actions = []
    if settings["on"]:

        def _single_viz_node_launch_actions(settings: Dict, suffix: Optional[int] = None) -> List:
            package = settings["pkg"]
            executable = settings["executable"]
            parameters_dict = get_parameters_dict(settings)

            # Get the other viz specific params
            urdf_file_name = "urdf/" + settings["urdf"]
            urdf_path = os.path.join(get_package_share_directory(settings["robot_pkg"]), urdf_file_name)
            parameters_dict["urdf_path_param"] = urdf_path
            if "tf_prefix" in settings:
                tf_prefix = settings["tf_prefix"] + "/"
            else:
                tf_prefix = ""

            parameters_dict["tf_prefix"] = tf_prefix
            launch_actions = []
            component_node = LifecycleNode(
                namespace="",
                name="obelisk_viz" if suffix is None else f"obelisk_viz_{suffix}",
                package=package,
                executable=executable,
                output="both",
                parameters=[parameters_dict],
            )
            launch_actions += [component_node]
            launch_actions += get_handlers(component_node, global_state_node, auto_start)

            # Read the URDF for the robot_state_publisher
            with open(urdf_path, "r") as urdf:
                robot_desc = urdf.read()

            # Get the topic where JointStates will be published to
            pub_settings = settings["publishers"]
            timer_settings = settings["timers"]

            robot_topic = "robot_description"
            if "robot_topic" in settings:
                robot_topic = settings["robot_topic"]

            launch_actions += [
                Node(
                    package="robot_state_publisher",
                    executable="robot_state_publisher",
                    name=f"robot_state_publisher_{suffix}",
                    output="screen",
                    parameters=[
                        {
                            "use_sim_time": False,
                            "robot_description": robot_desc,
                            "frame_prefix": tf_prefix,
                            "publish_frequency": 1.0 / timer_settings[0]["timer_period_sec"],
                        }
                    ],
                    remappings=[
                        (
                            "/joint_states",
                            pub_settings[0]["topic"],
                        ),  # remap the topic that the robot_state_publisher listens to
                        (
                            "/robot_description",
                            robot_topic,
                        ),  # remap the topic where the robot description is published to
                    ],
                )
            ]

            return launch_actions

        # Get the launch actions for each ObeliskViz node
        node_settings = settings["viz_nodes"]
        for i, viz_node_settings in enumerate(node_settings):
            launch_actions += _single_viz_node_launch_actions(viz_node_settings, suffix=i)

        # Setup Rviz
        if "rviz_config" not in settings.keys():
            launch_actions += [
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    output="screen",
                )
            ]
        else:
            rviz_file_name = "rviz/" + settings["rviz_config"]
            rviz_config_path = os.path.join(get_package_share_directory(settings["rviz_pkg"]), rviz_file_name)
            launch_actions += [
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    output="screen",
                    arguments=["-d", rviz_config_path],
                )
            ]

    return launch_actions


JOYSTICK_PARAMS = ["device_id", "device_name", "deadzone", "autorepeat_rate", "sticky_buttons", "coalesce_interval_ms"]
JOYSTICK_REMAPPINGS = {"pub_topic": "/joy", "sub_topic": "/joy/set_feedback"}

def get_launch_actions_from_joystick_settings(settings: Dict, global_state_node: LifecycleNode) -> List[LifecycleNode]:
    """Gets and configures all the launch actions related to joystick given the settings from the yaml."""
    launch_actions = []
    if settings["on"]:
        joystick_params = {}
        joystick_remappings = JOYSTICK_REMAPPINGS.copy()
        for setting_key, setting_val in settings.items():
            if setting_key == "on":
                continue
            if setting_key in JOYSTICK_PARAMS:
                joystick_params[setting_key] = setting_val
            elif setting_key in JOYSTICK_REMAPPINGS.keys():
                joystick_remappings[setting_key] = setting_val
            else:
                raise ValueError(f"Unknown joystick setting {setting_key}")

        launch_actions += [
            Node(
                package="joy",
                executable="joy_node",
                name="joy",
                output="screen",
                parameters=[
                    joystick_params
                ],
                remappings=[
                    (
                        "/joy",
                        joystick_remappings["pub_topic"],
                    ),  # remap the topic that the joystick publishes to
                    (
                        "/joy/set_feedback",
                        joystick_remappings["sub_topic"],
                    ),  # remap the topic where the joystick listens
                ],
            )
        ]

    return launch_actions


def get_handlers(component_node: LifecycleNode, global_state_node: LifecycleNode, auto_start: str = "false") -> List:
    """Gets all the handlers for the Lifecycle node.

    Parameters:
        component_node: the Obelisk component node whose lifecycle is being managed.
        global_state_node: the global state node all Obelisk nodes' lifecycle states match.
        auto_start: the bringup's (lowercased) auto_start launch argument. When "true"/"activate",
            the component's initial activation is keyed off ITS OWN configure completion rather
            than the global state node reaching `active` (see the activate handler comment).
    """
    # transition events to match the global state node
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(component_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )
    activate_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(component_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )
    deactivate_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(component_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_DEACTIVATE,
        )
    )
    cleanup_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(component_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CLEANUP,
        )
    )
    unconfigured_shutdown_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(component_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_UNCONFIGURED_SHUTDOWN,
        )
    )
    inactive_shutdown_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(component_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_INACTIVE_SHUTDOWN,
        )
    )
    active_shutdown_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(component_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVE_SHUTDOWN,
        )
    )

    # making event handlers
    configure_handler = RegisterEventHandler(
        launch_ros.event_handlers.on_state_transition.OnStateTransition(
            target_lifecycle_node=global_state_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[configure_event],
        )
    )
    if auto_start in ("true", "activate"):
        # Auto-start startup ordering: the global state node is a no-op, so it reaches `active`
        # within milliseconds of `inactive` -- usually before slow-starting component processes
        # have created their change_state services. launch delivers every emitted ChangeState on
        # its own service-waiting thread with no per-node ordering, so an activate keyed off the
        # global transition can reach a component BEFORE its still-undelivered configure, and an
        # rclpy component then dies on the invalid transition (activate from unconfigured). Key
        # the initial activation off the component's own configure completion instead, making the
        # configure -> activate order structural. handle_once so an obk-stop (cleanup) followed
        # by a re-configure doesn't self-activate; explicit re-activations are handled below.
        activate_handler = RegisterEventHandler(
            launch_ros.event_handlers.on_state_transition.OnStateTransition(
                target_lifecycle_node=component_node,
                start_state="configuring",
                goal_state="inactive",
                entities=[activate_event],
                handle_once=True,
            )
        )

        # Manual deactivate -> activate cycles (obk-activate) still propagate from the global
        # state node. The first global activation is the auto-start one, covered by the
        # self-chained handler above, so skip exactly that occurrence; by any later global
        # activation the component processes are alive with their services up, so the startup
        # ordering race cannot recur.
        global_activations = {"count": 0}

        def _reactivate(context: launch.LaunchContext) -> Optional[List]:
            global_activations["count"] += 1
            if global_activations["count"] == 1:
                return None
            return [activate_event]

        reactivate_handler = RegisterEventHandler(
            launch_ros.event_handlers.on_state_transition.OnStateTransition(
                target_lifecycle_node=global_state_node,
                start_state="activating",
                goal_state="active",
                entities=[OpaqueFunction(function=_reactivate)],
            )
        )
    else:
        # Without auto-start, activation only comes from an explicit operator transition of the
        # global state node, by which point the components have long been configured.
        activate_handler = RegisterEventHandler(
            launch_ros.event_handlers.on_state_transition.OnStateTransition(
                target_lifecycle_node=global_state_node,
                start_state="activating",
                goal_state="active",
                entities=[activate_event],
            )
        )
        reactivate_handler = None
    deactivate_handler = RegisterEventHandler(
        launch_ros.event_handlers.on_state_transition.OnStateTransition(
            target_lifecycle_node=global_state_node,
            start_state="deactivating",
            goal_state="inactive",
            entities=[deactivate_event],
        )
    )
    cleanup_handler = RegisterEventHandler(
        launch_ros.event_handlers.on_state_transition.OnStateTransition(
            target_lifecycle_node=global_state_node,
            start_state="cleaningup",
            goal_state="unconfigured",
            entities=[cleanup_event],
        )
    )
    unconfigured_shutdown_handler = RegisterEventHandler(
        launch_ros.event_handlers.on_state_transition.OnStateTransition(
            target_lifecycle_node=global_state_node,
            start_state="unconfigured",
            goal_state="shuttingdown",
            entities=[unconfigured_shutdown_event],
        )
    )
    inactive_shutdown_handler = RegisterEventHandler(
        launch_ros.event_handlers.on_state_transition.OnStateTransition(
            target_lifecycle_node=global_state_node,
            start_state="inactive",
            goal_state="shuttingdown",
            entities=[inactive_shutdown_event],
        )
    )
    active_shutdown_handler = RegisterEventHandler(
        launch_ros.event_handlers.on_state_transition.OnStateTransition(
            target_lifecycle_node=global_state_node,
            start_state="active",
            goal_state="shuttingdown",
            entities=[active_shutdown_event],
        )
    )
    launch_actions = [
        configure_handler,
        activate_handler,
        deactivate_handler,
        cleanup_handler,
        unconfigured_shutdown_handler,
        inactive_shutdown_handler,
        active_shutdown_handler,
    ]
    if reactivate_handler is not None:
        launch_actions.append(reactivate_handler)
    return launch_actions


def setup_logging_dir(config_name: str) -> str:
    """
    Configures the logging directory.

    Checks if there is already an obk_logs directory, and if not, create it.
    Then creates a folder inside of that will the date and the name of the config.
    Sets the ROS_LOG_DIR environment variable to this location.
    """
    # Check for the directory
    if "ROS_HOME" in os.environ:
        logging_ws = os.environ.get("ROS_HOME", "/home/")
    elif "OBELISK_ROOT" in os.environ:
        logging_ws = os.environ.get("OBELISK_ROOT", "/home/")
    else:
        raise EnvironmentError("OBELISK_ROOT and ROS_HOME are not set!")

    general_log_file_path = logging_ws + "/obk_logs"
    if not os.path.exists(general_log_file_path):
        os.makedirs(general_log_file_path)

    # Get current date and time and replace space with underscore
    now = datetime.now()
    curr_date_time = now.strftime("%Y%m%d_%H%M%S")

    # Now make a folder for this specific run
    run_log_file_path = general_log_file_path + "/" + config_name + "_" + curr_date_time
    os.makedirs(run_log_file_path)

    # Set the ROS environment variable
    os.environ["ROS_LOG_DIR"] = run_log_file_path

    return run_log_file_path
