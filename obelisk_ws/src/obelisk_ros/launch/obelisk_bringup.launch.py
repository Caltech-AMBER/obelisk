import os
from typing import Dict, List, Tuple

import launch
import launch_ros
import lifecycle_msgs.msg
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from rclpy.impl import rcutils_logger

from obelisk_py.core.utils.launch_utils import (
    get_launch_actions_from_node_settings,
    load_config_file,
    setup_logging_dir,
)

os.environ["ROS_LOG_DIR"] = "/home/zolkin/AmberLab/Project-Obelisk/obelisk/obk_logs/debug_dummy"

logger = rcutils_logger.RcutilsLogger(name="obelisk_bringup")


def launch_args_setup(context: launch.LaunchContext, *args: List, **kwargs: Dict) -> Tuple[List, Dict]:
    """Sets up all launch arguments at once."""
    launch_actions = []
    launch_args = {}

    # expose path to config file
    config_file_path = LaunchConfiguration("config_file_path")
    device_name = LaunchConfiguration("device_name")
    auto_start = LaunchConfiguration("auto_start")
    bag = LaunchConfiguration("bag")

    config_file_path_arg = DeclareLaunchArgument("config_file_path")
    device_name_arg = DeclareLaunchArgument("device_name")
    auto_start_arg = DeclareLaunchArgument("auto_start", default_value="True")
    bag_arg = DeclareLaunchArgument("bag", default_value="True")

    launch_actions += [config_file_path_arg, device_name_arg, auto_start_arg, bag_arg]
    launch_args.update(
        {
            "config_file_path": config_file_path,
            "device_name": device_name,
            "auto_start": auto_start,
            "bag": bag,
        }
    )

    return launch_actions, launch_args


def obelisk_setup(context: launch.LaunchContext, launch_args: Dict) -> List:
    """Returns the launch actions associated with all the nodes in the Obelisk architecture."""
    # Reset config to adjust the logging directory
    launch.logging.launch_config.reset()

    # parsing the launch arguments
    config_file_path = context.launch_configurations.get("config_file_path")
    device_name = context.launch_configurations.get("device_name")
    auto_start = context.launch_configurations.get("auto_start")
    auto_start = "true" if auto_start is None else auto_start.lower()
    bag = context.launch_configurations.get("bag")
    bag = "true" if bag is None else bag.lower()

    full_config_dict = load_config_file(config_file_path)
    config_name = full_config_dict["config"]
    obelisk_config = full_config_dict[device_name]  # grab the settings associated with the device
    logger.info(f"Bringing up the Obelisk nodes on device {device_name}...")

    # checks - we must at least have these 3 components
    assert "control" in obelisk_config
    assert "estimation" in obelisk_config
    assert "robot" in obelisk_config
    obelisk_launch_actions = []

    # Setup logging
    run_log_file_path = setup_logging_dir(config_name)
    logger.info(f"Logging directory set to {run_log_file_path}")

    if bag.lower() == "true":
        bag_path = run_log_file_path + "/obk_stack_bag"
        obelisk_launch_actions += [ExecuteProcess(cmd=["ros2", "bag", "record", "-a", "-o", bag_path], output="both")]

    # create global state node
    global_state_node = LifecycleNode(
        namespace="", package="obelisk_ros", executable="global_state", name=config_name, output="both"
    )
    shutdown_event = EmitEvent(event=launch.events.Shutdown())
    shutdown_event_handler = RegisterEventHandler(
        launch_ros.event_handlers.on_state_transition.OnStateTransition(
            target_lifecycle_node=global_state_node,
            goal_state="finalized",
            entities=[shutdown_event],
        )
    )  # when the global state node enters its shutdown state, the launch file also shuts down
    obelisk_launch_actions += [global_state_node, shutdown_event_handler]
    if auto_start:
        configure_event = EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=launch.events.matches_action(global_state_node),
                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
            )
        )
        activate_event = EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=launch.events.matches_action(global_state_node),
                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
            )
        )
        activate_upon_configure_handler = RegisterEventHandler(
            launch_ros.event_handlers.on_state_transition.OnStateTransition(
                target_lifecycle_node=global_state_node,
                start_state="configuring",
                goal_state="inactive",
                entities=[activate_event],
            )
        )  # once the node is configured, it will be activated automatically
        obelisk_launch_actions += [configure_event, activate_upon_configure_handler]

    # generate all launch actions
    obelisk_launch_actions += get_launch_actions_from_node_settings(
        obelisk_config["control"],
        "control",
        global_state_node,
    )
    obelisk_launch_actions += get_launch_actions_from_node_settings(
        obelisk_config["estimation"],
        "estimation",
        global_state_node,
    )
    obelisk_launch_actions += get_launch_actions_from_node_settings(
        obelisk_config["robot"],
        "robot",
        global_state_node,
    )
    if "sensors" in obelisk_config:
        obelisk_launch_actions += get_launch_actions_from_node_settings(
            obelisk_config["sensing"],
            "sensing",
            global_state_node,
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
