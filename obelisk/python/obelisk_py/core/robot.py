from abc import ABC, abstractmethod

import obelisk_sensor_msgs.msg as osm
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

from obelisk_py.core.node import ObeliskNode
from obelisk_py.core.obelisk_typing import ObeliskControlMsg


class ObeliskRobot(ABC, ObeliskNode):
    """Abstract Obelisk robot node.

    Obelisk robots are representations of the physical robot. They take in Obelisk control messages and can optionally
    output Obelisk sensor messages. We expect code in this function to communicate with the low-level control interface
    of a real system.

    [NOTE] In derived classes, you should declare settings for sensor publishers.
    """

    def __init__(self, node_name: str) -> None:
        """Initialize the Obelisk robot."""
        super().__init__(node_name)
        self.register_obk_subscription(
            "sub_ctrl_setting",
            self.apply_control,
            key="subscriber_ctrl",
            msg_type=None,  # generic, specified in config file
        )

    @abstractmethod
    def apply_control(self, control_msg: ObeliskControlMsg) -> None:
        """Apply the control message to the robot.

        Code interfacing with the hardware should be implemented here.

        Parameters:
            control_msg: Obelisk control message to apply to the robot.
        """


class ObeliskSimRobot(ObeliskRobot):
    """Abstract Obelisk simulated robot node.

    Simulated robots can be seen as a special case of the hardware robot, where all the sensors used to control the
    robot are contained in the simulator and privileged information about the system can be published directly for
    logging or debugging purposes. This privileged information is known as the TrueSimState of the simulator.

    Each ObeliskSimRobot is associated with a simulator. For instance, we currently support MuJoCo, but there is nothing
    preventing the end user from implementing their own simulator of choice or us from implementing other simulators.
    """

    def __init__(self, node_name: str) -> None:
        """Initialize the Obelisk sim robot."""
        super().__init__(node_name)
        self.register_obk_timer(
            "timer_true_sim_state_setting",
            self.publish_true_sim_state,
            key="timer_true_sim_state",
            default_config_str="",
        )
        self.register_obk_publisher(
            "pub_true_sim_state_setting",
            key="publisher_true_sim_state",
            msg_type=osm.TrueSimState,
            default_config_str="",
        )

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the simulation."""
        super().on_configure(state)

        # checking the settings of the true sim state pub/timer
        if "publisher_true_sim_state" in self.obk_publishers and "timer_true_sim_state" in self.obk_timers:
            assert (
                self.obk_timers["timer_true_sim_state"].callback == self.publish_true_sim_state
            ), f"Timer callback must be publish_true_sim_state! Is {self.obk_timers['timer_true_sim_state'].callback}."
        else:
            self.timer_true_sim_state = None
            self.publisher_true_sim_state = None

        return TransitionCallbackReturn.SUCCESS

    def publish_true_sim_state(self) -> osm.TrueSimState:
        """Publish the TrueSimState of the simulator.

        This is the timer callback that publishes the TrueSimState and is expected to call self.publisher_true_sim_state
        internally. Note that the true sim state message is still returned afterwards, mostly for logging/debugging
        purposes. The publish call is the important part, NOT the returned value, since the topic is what the
        ObeliskEstimator subscribes to.

        Returns:
            obelisk_true_sim_state_msg: An Obelisk message type containing the true sim state.
        """
        raise NotImplementedError

    @abstractmethod
    def run_simulator(self) -> None:
        """Run the simulator.

        The control input into the simulator is accessed through the shared control array. If any simulator
        configuration needs to occur, then the on_configure function should be overridden (and this implementation
        should be called with super()). This function should run the simulator loop and update the state of the
        simulator as long as the node is active.
        """