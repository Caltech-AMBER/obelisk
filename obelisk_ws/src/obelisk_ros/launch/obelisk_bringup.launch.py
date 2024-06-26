from typing import Dict, List, Tuple

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from rclpy.impl import rcutils_logger

logger = rcutils_logger.RcutilsLogger(name="obelisk_bringup")


def launch_args_setup(context: launch.LaunchContext, *args: List, **kwargs: Dict) -> Tuple[List, Dict]:
    """Sets up all launch arguments at once."""
    launch_actions = []
    launch_args = {}

    # expose path to config file
    config_file_path = LaunchConfiguration("config_file_path")
    config_file_path_arg = DeclareLaunchArgument("config_file_path")

    launch_actions += [config_file_path_arg]
    launch_args.update({"config_file_path": config_file_path})

    return launch_actions, launch_args


def obelisk_setup(context: launch.LaunchContext, launch_args: Dict) -> List:
    """Returns the launch actions associated with all the nodes in the Obelisk architecture."""
    obelisk_launch_actions = []

    # parsing the config file
    config_file_path = launch_args["config_file_path"]  # noqa
    # TODO(ahl): parse the config file, get all config strings for every part of the stack
    # TODO(ahl): create all the launch actions for LifecycleNodes, pass config strings
    # TODO(ahl): add all the launch actions to obelisk_launch_actions in loops
    return obelisk_launch_actions  # TODO(ahl): return the launch actions


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
