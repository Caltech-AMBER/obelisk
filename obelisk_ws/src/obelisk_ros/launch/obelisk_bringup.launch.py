from typing import Dict, List, Tuple

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from rclpy.impl import rcutils_logger

from obelisk_py.utils.launch_utils import get_launch_actions_from_node_settings, load_config_file

logger = rcutils_logger.RcutilsLogger(name="obelisk_bringup")


def launch_args_setup(context: launch.LaunchContext, *args: List, **kwargs: Dict) -> Tuple[List, Dict]:
    """Sets up all launch arguments at once."""
    launch_actions = []
    launch_args = {}

    # expose path to config file
    config_file_path = LaunchConfiguration("config_file_path")
    device_name = LaunchConfiguration("device_name")
    auto_start = LaunchConfiguration("auto_start")

    config_file_path_arg = DeclareLaunchArgument("config_file_path")
    device_name_arg = DeclareLaunchArgument("device_name")
    auto_start_arg = DeclareLaunchArgument("auto_start", default_value="True")

    launch_actions += [config_file_path_arg, device_name_arg, auto_start_arg]
    launch_args.update(
        {
            "config_file_path": config_file_path,
            "device_name": device_name,
            "auto_start": auto_start,
        }
    )

    return launch_actions, launch_args


def obelisk_setup(context: launch.LaunchContext, launch_args: Dict) -> List:
    """Returns the launch actions associated with all the nodes in the Obelisk architecture."""
    # parsing the launch arguments
    config_file_path = context.launch_configurations.get("config_file_path")
    device_name = context.launch_configurations.get("device_name")
    auto_start = context.launch_configurations.get("auto_start")
    obelisk_config = load_config_file(config_file_path)[device_name]  # grab the settings associated with the device
    logger.info(f"Bringing up the Obelisk nodes on device {device_name}...")

    # checks - we must at least have these 3 components
    assert "control" in obelisk_config
    assert "estimation" in obelisk_config
    assert "robot" in obelisk_config

    # generate all launch actions
    obelisk_launch_actions = get_launch_actions_from_node_settings(
        obelisk_config["control"],
        "control",
        auto_start=auto_start,
    )
    obelisk_launch_actions += get_launch_actions_from_node_settings(
        obelisk_config["estimation"],
        "estimation",
        auto_start=auto_start,
    )
    obelisk_launch_actions += get_launch_actions_from_node_settings(
        obelisk_config["robot"],
        "robot",
        auto_start=auto_start,
    )
    if "sensors" in obelisk_config:
        obelisk_launch_actions += get_launch_actions_from_node_settings(
            obelisk_config["sensing"],
            "sensing",
            auto_start=auto_start,
        )
    return obelisk_launch_actions


def launch_setup(context: launch.LaunchContext, *args: List, **kwargs: Dict) -> List:
    """Collates all the launch actions for each part of the pipeline."""
    launch_actions, launch_args = launch_args_setup(context, *args, **kwargs)
    launch_actions += obelisk_setup(context, launch_args)
    return launch_actions


def generate_launch_description() -> LaunchDescription:
    """Generates the launch description.

    Uses the OpaqueFunction to allow for analyzing and manipulating the
    launch arguments that get passed in before running the relevant actions.
    """
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
