from pathlib import Path
from typing import Dict, List, Optional, Union

import launch
import launch_ros
import lifecycle_msgs.msg
from ament_index_python.packages import get_package_share_directory
from launch.actions import EmitEvent, RegisterEventHandler
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from ruamel.yaml import YAML


def load_config_file(file_path: Union[str, Path], package_name: Optional[str] = None) -> Dict:
    """Loads an Obelisk configuration file.

    Parameters:
        file_path: the path to the configuration file, either absolute or relative to the 'config' directory of an
            installed ROS2 package.
        package_name: the name of the ROS2 package that contains the configuration file. If not provided, the
            'obelisk_ros' package is assumed.

    Returns:
        A dictionary containing the configuration settings.
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

    # iterate over the settings for each component
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
    callback_group_settings = ",".join([f"{k}:{v}" for k, v in node_settings["callback_groups"].items()])
    pub_settings_dict = (
        get_component_settings_subdict(node_settings, "publishers") if "publishers" in node_settings else {}
    )
    sub_settings_dict = (
        get_component_settings_subdict(node_settings, "subscribers") if "subscribers" in node_settings else {}
    )
    timer_settings_dict = get_component_settings_subdict(node_settings, "timers") if "timers" in node_settings else {}
    sim_settings_dict = get_component_settings_subdict(node_settings, "sim") if "sim" in node_settings else {}

    # create parameters dictionary for a node by combining all the settings
    parameters_dict = {"callback_group_settings": callback_group_settings}
    parameters_dict.update(pub_settings_dict)
    parameters_dict.update(sub_settings_dict)
    parameters_dict.update(timer_settings_dict)
    parameters_dict.update(sim_settings_dict)
    return parameters_dict


def get_launch_actions_from_node_settings(
    node_settings: Dict,
    node_type: str,
    auto_start: Optional[bool] = True,
) -> List[LifecycleNode]:
    """Returns the launch actions associated with a node.

    Parameters:
        node_settings: the settings dictionary of an Obelisk node.
        node_type: the type of the node.
        auto_start: whether the node should be started automatically.

    Returns:
        A list of launch actions.
    """
    assert node_type in ["control", "estimation", "robot", "sensing"]

    def _single_component_launch_actions(node_settings: Dict, suffix: Optional[int] = None) -> List:
        impl = node_settings["impl"]
        executable = node_settings["executable"]
        if node_type == "robot" and str(node_settings["is_simulated"]).lower() == "true":
            package = f"obelisk_sim_{'py' if impl == 'python' else 'cpp'}"
        else:
            package = f"obelisk_{node_type}_{'py' if impl == 'python' else 'cpp'}"

        launch_actions = []
        parameters_dict = get_parameters_dict(node_settings)
        component_node = LifecycleNode(
            namespace="",
            name=f"obelisk_{node_type}" if suffix is None else f"obelisk_{node_type}_{suffix}",
            package=package,
            executable=executable,
            output="screen",
            parameters=[parameters_dict],
        )
        launch_actions += [component_node]
        if auto_start:
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
            activate_upon_configure_handler = RegisterEventHandler(
                launch_ros.event_handlers.on_state_transition.OnStateTransition(
                    target_lifecycle_node=component_node,
                    goal_state="inactive",  # once the node is configured, it will be activated automatically
                    entities=[configure_event, activate_event],
                )
            )
            launch_actions += [configure_event, activate_upon_configure_handler]
        return launch_actions

    if node_type == "sensing":
        node_launch_actions = []
        for i, sensor_settings in enumerate(node_settings):
            node_launch_actions += _single_component_launch_actions(sensor_settings, suffix=i)
        return node_launch_actions
    else:
        return _single_component_launch_actions(node_settings)
