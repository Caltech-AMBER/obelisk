import os
import re
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Union

import launch
import launch_ros
import lifecycle_msgs.msg
from ament_index_python.packages import get_package_share_directory
from launch.actions import EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch_ros.actions import LifecycleNode, Node
from launch_ros.events.lifecycle import ChangeState
from ruamel.yaml import YAML

from launch.events import matches_action
from launch.event_handlers import OnProcessStart
from launch_ros.event_handlers.on_state_transition import OnStateTransition
from lifecycle_msgs.msg import Transition

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


def get_component_settings_subdict(node_settings: Dict, subdict_name: str) -> Dict:
    """Returns a subdictionary of the component settings associated with an Obelisk node.

    Parameters:
        node_settings: the settings dictionary of an Obelisk node.
        subdict_name: the name of the subdictionary to extract from the node settings.

    Returns:
        A dictionary with the ROS parameter names as keys and the corresponding settings as values.
    """
    component_settings_subdict = {}

    # iterate over the settings for each component - if doesn't exist, make a new dict entry, else append it
    for component_settings in node_settings[subdict_name]:
        if component_settings["ros_parameter"] not in component_settings_subdict:
            component_settings_subdict[component_settings["ros_parameter"]] = [
                ",".join([f"{k}:{v}" for k, v in component_settings.items() if k != "ros_parameter"])
            ]
        else:
            component_settings_subdict[component_settings["ros_parameter"]].append(
                ",".join([f"{k}:{v}" for k, v in component_settings.items() if k != "ros_parameter"])
            )

    # if there is only one setting, don't return a list, just a single element
    for ros_parameter, pub_settings_list in component_settings_subdict.items():
        if len(pub_settings_list) == 1:
            component_settings_subdict[ros_parameter] = pub_settings_list[0]

    return component_settings_subdict


def get_component_sim_settings_subdict(node_settings: Dict) -> Dict:
    """Returns a subdictionary of the simulation settings associated with an Obelisk node.

    Parameters:
        node_settings: the settings dictionary of an Obelisk node.

    Returns:
        A dictionary with the ROS parameter names as keys and the corresponding settings as values.
    """
    sim_settings_dict = {}
    for sim_settings in node_settings["sim"]:
        # convert the dictionary of simulation settings to a string, excluding 'ros_parameter'
        sim_settings_strs = []
        for k, v in sim_settings.items():
            if k != "ros_parameter":
                sim_settings_strs.append(f"{k}:{v}")
        sim_settings_str = ",".join(sim_settings_strs).replace(" ", "")

        # replace colons in 'sensor_names':{...} with '$', delete curly braces only within the outermost braces
        def _replace_colons_delete_inner_braces(match: re.Match) -> str:
            dollar_str = match.group(0).replace(":", "$")
            dollar_str = dollar_str.replace("$", ":", 1)

            # find the position of the outermost braces
            open_brace = dollar_str.find("{")
            close_brace = dollar_str.rfind("}")

            if open_brace != -1 and close_brace != -1 and open_brace < close_brace:
                # keep content before the first brace and after the last brace
                prefix = dollar_str[: open_brace + 1]
                suffix = dollar_str[close_brace:]

                # remove braces from the inner content
                inner_content = dollar_str[open_brace + 1 : close_brace]
                inner_content_no_braces = inner_content.replace("{", "").replace("}", "")

                return f"{prefix}{inner_content_no_braces}{suffix}"
            else:
                # if we can't find matching outermost braces, return the original string (SHOULD NEVER HAPPEN)
                return dollar_str

        sim_settings_str = re.sub(r"'sensor_names':\{[^\}]*\}", _replace_colons_delete_inner_braces, sim_settings_str)

        # replace commas in 'sensor_names':{...} with '&', remove outermost braces
        def _replace_commas_delete_outer_braces(match: re.Match) -> str:
            return match.group(0).replace(",", "&").replace("{", "").replace("}", "")

        sim_settings_str = re.sub(r"'sensor_names':\{[^\}]*\}", _replace_commas_delete_outer_braces, sim_settings_str)

        # remove single quotes
        sim_settings_str = sim_settings_str.replace("'", "")

        # replace commas between matching curly braces with "|"
        def _replace_commas_between_curly_braces(match: re.Match) -> str:
            return match.group(0).replace(",", "|")

        sim_settings_str = re.sub(r"\{[^{}]*\}", _replace_commas_between_curly_braces, sim_settings_str)

        # replace commas in 'sensor_settings':[...,...] with "+"
        def _replace_commas_in_sensor_settings(match: re.Match) -> str:
            return match.group(0).replace(",", "+")

        sim_settings_str = re.sub(r"sensor_settings:\[.*?\]", _replace_commas_in_sensor_settings, sim_settings_str)

        # replace colons inside 'sensor_settings':[...:...] with "="
        def _replace_colons_in_sensor_settings(match: re.Match) -> str:
            content = match.group(0)
            return re.sub(r"(?<=\[).*?(?=\])", lambda m: m.group(0).replace(":", "="), content)

        sim_settings_str = re.sub(r"sensor_settings:\[.*?\]", _replace_colons_in_sensor_settings, sim_settings_str)

        # add the formatted string to the dictionary with the ROS parameter as the key
        sim_settings_dict[sim_settings["ros_parameter"]] = sim_settings_str

    return sim_settings_dict


def get_parameters_dict(node_settings: Dict) -> Dict:
    """Returns the parameters dictionary corresponding to the settings of a node in the Obelisk architecture.

    Parameters:
        node_settings: the settings dictionary of an Obelisk node.

    Returns:
        A dictionary with the ROS parameter names as keys and the corresponding settings as values.
    """
    if node_settings is None:
        return {}

    # parse settings for each component
    if "callback_groups" in node_settings:
        callback_group_settings = ",".join([f"{k}:{v}" for k, v in node_settings["callback_groups"].items()])
    else:
        callback_group_settings = None
    pub_settings_dict = (
        get_component_settings_subdict(node_settings, "publishers") if "publishers" in node_settings else {}
    )
    sub_settings_dict = (
        get_component_settings_subdict(node_settings, "subscribers") if "subscribers" in node_settings else {}
    )
    timer_settings_dict = get_component_settings_subdict(node_settings, "timers") if "timers" in node_settings else {}
    sim_settings_dict = get_component_sim_settings_subdict(node_settings) if "sim" in node_settings else {}

    # create parameters dictionary for a node by combining all the settings
    parameters_dict = (
        {"callback_group_settings": callback_group_settings} if callback_group_settings is not None else {}
    )
    if "params_path" in node_settings:
        parameters_dict["params_path"] = node_settings["params_path"]
    if "params" in node_settings:
        parameters_dict.update(node_settings["params"])
    parameters_dict.update(pub_settings_dict)
    parameters_dict.update(sub_settings_dict)
    parameters_dict.update(timer_settings_dict)
    parameters_dict.update(sim_settings_dict)
    return parameters_dict


def get_launch_actions_from_node_settings(
    node_settings: Dict,
    node_type: str,
    global_state_node: Optional[LifecycleNode],
) -> List[LifecycleNode]:
    """Returns the launch actions associated with a node.

    Parameters:
        node_settings: the settings dictionary of an Obelisk node.
        node_type: the type of the node.
        global_state_node: the global state node. All Obelisk nodes' lifecycle 
        states match this node's state. If `global_state_node` is None, the nodes
        will automatically go through the lifecycle after configuration.

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
        launch_actions += get_handlers(component_node, global_state_node=global_state_node)
        return launch_actions

    node_launch_actions = []
    for i, node_setting in enumerate(node_settings):
        node_launch_actions += _single_component_launch_actions(node_setting, suffix=i)
    return node_launch_actions


def get_launch_actions_from_viz_settings(
        settings: Dict, 
        global_state_node: Optional[LifecycleNode]
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
            launch_actions += get_handlers(component_node, 
                                           global_state_node=global_state_node)

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

        if "viz_tool" not in settings or settings["viz_tool"] == "rviz":
            # Setup Rviz
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
        elif settings["viz_tool"] == "foxglove":
            # setup fox glove
            xml_launch_file = os.path.join(
                get_package_share_directory("foxglove_bridge"), "launch", "foxglove_bridge_launch.xml"
            )
            launch_actions += [
                IncludeLaunchDescription(
                    FrontendLaunchDescriptionSource(xml_launch_file),
                    launch_arguments={
                        "capabilities": "[clientPublish,parametersSubscribe,services,connectionGraph,assets]"
                    }.items(),
                )
            ]
        else:
            raise RuntimeError("Invalid setting for `viz_tool`. Must be either `rviz` of `foxglove`.")

    return launch_actions


def get_launch_actions_from_joystick_settings(settings: Dict) -> List[LifecycleNode]:
    """Gets and configures all the launch actions related to joystick given the settings from the yaml."""
    launch_actions = []
    if settings["on"]:
        # TODO: I think there is a better way to handle these defaults, i.e., not even passing the parameter if nothing
        # is supplied.
        device_id = settings["device_id"] if "device_id" in settings else 0

        device_name = settings["device_name"] if "device_name" in settings else ""

        deadzone = settings["deadzone"] if "deadzone" in settings else 0.05

        autorepeat_rate = settings["autorepeat_rate"] if "autorepeat_rate" in settings else 20.0

        sticky_buttons = settings["sticky_buttons"] if "sticky_buttons" in settings else False

        coalesce_interval_ms = settings["coalesce_interval_ms"] if "coalesce_interval_ms" in settings else 1

        pub_topic = settings["pub_topic"] if "pub_topic" in settings else "/joy"

        sub_topic = settings["sub_topic"] if "sub_topic" in settings else "/joy/set_feedback"

        launch_actions += [
            Node(
                package="joy",
                executable="joy_node",
                name="joy",
                output="screen",
                parameters=[
                    {
                        "device_id": device_id,
                        "device_name": device_name,
                        "deadzone": deadzone,
                        "autorepeat_rate": autorepeat_rate,
                        "sticky_buttons": sticky_buttons,
                        "coalesce_interval_ms": coalesce_interval_ms,
                    }
                ],
                remappings=[
                    (
                        "/joy",
                        pub_topic,
                    ),  # remap the topic that the joystick publishes to
                    (
                        "/joy/set_feedback",
                        sub_topic,
                    ),  # remap the topic where the joystick listens
                ],
            )
        ]

    return launch_actions


def get_handlers(component_node: LifecycleNode, global_state_node: Optional[LifecycleNode]) -> List:
    """Gets all the handlers for the Lifecycle node."""
    lifecycle_node_matcher = matches_action(component_node)

    # Define transition events with their IDs
    transitions = {
        "configure": Transition.TRANSITION_CONFIGURE,
        "activate": Transition.TRANSITION_ACTIVATE,
        "deactivate": Transition.TRANSITION_DEACTIVATE,
        "cleanup": Transition.TRANSITION_CLEANUP,
        "unconfigured_shutdown": Transition.TRANSITION_UNCONFIGURED_SHUTDOWN,
        "inactive_shutdown": Transition.TRANSITION_INACTIVE_SHUTDOWN,
        "active_shutdown": Transition.TRANSITION_ACTIVE_SHUTDOWN,
    }

    events = {
        name: EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=lifecycle_node_matcher,
                transition_id=transition_id
            )
        )
        for name, transition_id in transitions.items()
    }

    def make_handler(start: str, goal: str, event_name: str, target: LifecycleNode):
        """
        When the `target_lifecycle_node` transitions from `start` to `goal`,
        emit the event `event_name` to the component node.
        """  # noqa: D205
        return RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=target,
                start_state=start,
                goal_state=goal,
                entities=[events[event_name]]
            )
        )
    
    # Event handlers for the component node depend solely on the state of the 
    # component node
    if not global_state_node:
        target = component_node
        handlers = [
            RegisterEventHandler(
                OnProcessStart(
                    target_action=target,
                    on_start=[events["configure"]],
                )
            ),
            make_handler("configuring", "inactive", "activate", target),
            make_handler("active", "deactivating", "deactivate", target),
            make_handler("inactive", "cleaningup", "cleanup", target),
        ]
    else:
        # Event handlers will transition the component node's state to match 
        # that of the global state node
        target = global_state_node
        handlers = [
            make_handler("configuring", "inactive", "configure", target),
            make_handler("activating", "active", "activate", target),
            make_handler("deactivating", "inactive", "deactivate", target),
            make_handler("cleaningup", "unconfigured", "cleanup", target),
            
        ]
    # Add shutting down handlers
    handlers += [
        make_handler("unconfigured", "shuttingdown", "unconfigured_shutdown", target),
        make_handler("inactive", "shuttingdown", "inactive_shutdown", target),
        make_handler("active", "shuttingdown", "active_shutdown", target),
    ]
    return handlers


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
