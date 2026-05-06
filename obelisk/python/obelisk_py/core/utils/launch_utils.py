import os
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Union

import launch
import launch_ros
import lifecycle_msgs.msg
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import EmitEvent, RegisterEventHandler
from launch_ros.actions import LifecycleNode, Node
from launch_ros.events.lifecycle import ChangeState
from ruamel.yaml import YAML

# Sections of a node's settings dict that Obelisk owns and bundles into the single
# `obelisk_settings` ROS parameter at launch time.
_OBELISK_SETTINGS_KEYS = ("publishers", "subscribers", "timers", "sim", "callback_groups")


def load_config_file(file_path: Union[str, Path], package_name: Optional[str] = None) -> Dict:
    """Loads an Obelisk configuration file.

    Parameters:
        file_path: the path to the configuration file, either absolute or relative to the 'config' directory of an
            installed ROS2 package.
        package_name: the name of the ROS2 package that contains the configuration file. If not provided, the
            'obelisk_ros' package is assumed.

    Returns:
        config: A dictionary containing the configuration settings.
    """
    file_path = str(file_path)
    if not file_path.startswith("/"):
        try:
            obk_ros_dir = get_package_share_directory("obelisk_ros" if package_name is None else package_name)
            abs_file_path = Path(obk_ros_dir) / "config" / file_path
        except Exception as e:
            raise FileNotFoundError(
                f"Could not find package {package_name}. If you supply a relative path to the config file, it must lie "
                "in a subdirectory named 'config' of either the obelisk_ros package or some other ROS2 package of your "
                "choice. Otherwise, you can supply an absolute path to the config file."
            ) from e
    else:
        abs_file_path = Path(file_path)

    yaml = YAML(typ="safe")
    try:
        return yaml.load(abs_file_path)
    except Exception as e:
        raise FileNotFoundError(f"Could not load a configuration file at {abs_file_path}!") from e


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
) -> List[LifecycleNode]:
    """Returns the launch actions associated with a node.

    Parameters:
        node_settings: the settings dictionary of an Obelisk node.
        node_type: the type of the node.
        global_state_node: the global state node. All Obelisk nodes' lifecycle states match this node's state.

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
        launch_actions += get_handlers(component_node, global_state_node)
        return launch_actions

    node_launch_actions = []
    for i, node_setting in enumerate(node_settings):
        node_launch_actions += _single_component_launch_actions(node_setting, suffix=i)
    return node_launch_actions


def get_launch_actions_from_viz_settings(settings: Dict, global_state_node: LifecycleNode) -> List[LifecycleNode]:
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
            launch_actions += get_handlers(component_node, global_state_node)

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


def get_handlers(component_node: LifecycleNode, global_state_node: LifecycleNode) -> List:
    """Gets all the handlers for the Lifecycle node."""
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
    activate_handler = RegisterEventHandler(
        launch_ros.event_handlers.on_state_transition.OnStateTransition(
            target_lifecycle_node=global_state_node,
            start_state="activating",
            goal_state="active",
            entities=[activate_event],
        )
    )
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
