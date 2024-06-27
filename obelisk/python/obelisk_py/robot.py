import ctypes
import multiprocessing
from abc import ABC, abstractmethod
from typing import List

import obelisk_sensor_msgs.msg as osm
import rclpy
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

from obelisk_py.node import ObeliskNode
from obelisk_py.obelisk_typing import ObeliskControlMsg


class ObeliskRobot(ABC, ObeliskNode):
    """Abstract Obelisk robot node.

    Obelisk robots are representations of the physical robot. They take in Obelisk control messages and can optionally
    output Obelisk sensor messages. We expect code in this function to communicate with the low-level control interface
    of a real system.
    """

    def __init__(self, node_name: str) -> None:
        """Initialize the Obelisk robot."""
        super().__init__(node_name)
        self.declare_parameter("sub_ctrl_setting", rclpy.Parameter.Type.STRING)
        self.declare_parameter("pub_sensor_settings", rclpy.Parameter.Type.STRING_ARRAY)

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the robot."""
        super().on_configure(state)

        # parsing config strings
        self.sub_ctrl_setting = self.get_parameter("sub_ctrl_setting").get_parameter_value().string_value
        self.pub_sensor_settings = self.get_parameter("pub_sensor_settings").get_parameter_value().string_array_value

        # create publishers and subscriber
        self.subscriber_ctrl = self._create_subscription_from_config_str(self.sub_ctrl_setting, self.apply_control)
        self.publisher_sensors = []
        for sensor_setting in self.pub_sensor_settings:
            pub_sensor = self._create_publisher_from_config_str(sensor_setting)
            self.publisher_sensors.append(pub_sensor)

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Clean up the robot."""
        super().on_cleanup(state)

        # destroy publishers + config strings
        self.subscriber_ctrl.destroy()
        for sensor_publisher in self.publisher_sensors:
            self.destroy_publisher(sensor_publisher)

        del self.subscriber_ctrl
        del self.publisher_sensors
        del self.sub_ctrl_setting
        del self.pub_sensor_settings
        return TransitionCallbackReturn.SUCCESS

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
        self.declare_parameter("n_u", rclpy.Parameter.Type.INTEGER)  # control input dimension
        self.declare_parameter("timer_true_sim_state_setting", rclpy.Parameter.Type.STRING)
        self.declare_parameter("pub_true_sim_state_setting", rclpy.Parameter.Type.STRING)

    def _set_shared_ctrl(self, ctrl: List[float]) -> None:
        """Set the shared control array.

        This is a convenience function to set the shared control array in a thread-safe manner. This should be called in
        the apply_control function. The shared control array is used to communicate the control input to the simulator.

        The values can be accessed in the simulator by running
        ```
        with self.lock:
            ctrl = list(self.shared_ctrl)
        ```

        Parameters:
            ctrl: The control array of length n_u.
        """
        with self.lock:
            self.shared_ctrl[:] = [ctypes.c_float(value) for value in ctrl]

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the simulation."""
        super().on_configure(state)

        # non-config string parameters
        self.n_u = self.get_parameter("n_u").get_parameter_value().integer_value
        assert self.n_u > 0, "Control input dimension must be positive!"

        # parsing config strings
        self.timer_true_sim_state_setting = (
            self.get_parameter("timer_true_sim_state_setting").get_parameter_value().string_value
        )
        self.pub_true_sim_state_setting = (
            self.get_parameter("pub_true_sim_state_setting").get_parameter_value().string_value
        )

        if self.pub_true_sim_state_setting != [""]:
            self.timer_true_sim_state = self._create_timer_from_config_str(
                self.timer_true_sim_state_setting,
                self.publish_true_sim_state,
            )
            self.publisher_true_sim_state = self._create_publisher_from_config_str(self.pub_true_sim_state_setting)

            # checks
            assert (
                self.timer_true_sim_state.callback == self.publish_true_sim_state
            ), f"Timer callback must be publish_true_sim_state! Is {self.timer_true_sim_state.callback}."
        else:
            self.get_logger().warn("No true sim state publisher configured!")
            self.timer_true_sim_state = None
            self.publisher_true_sim_state = None

        # setting up the simulator
        self.shared_ctrl = multiprocessing.Array(ctypes.c_float, self.n_u, lock=True)
        self.lock = self.shared_ctrl.get_lock()
        self.sim_process = multiprocessing.Process(target=self.run_simulator)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate the simulation.

        The abstract simulation loop is started here. If the user wants to implement features like pausing the sim,
        they must implement this themselves in the on_deactivate function.
        """
        super().on_activate(state)
        self.sim_process.start()  # starts the simulator
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Clean up the simulation."""
        super().on_cleanup(state)

        # terminate the simulation process
        if self.sim_process.is_alive():
            self.sim_process.terminate()
            self.sim_process.join()

        # destroy publishers + timers
        if self.timer_true_sim_state is not None and self.publisher_true_sim_state is not None:
            self.destroy_timer(self.timer_true_sim_state)
            self.destroy_publisher(self.publisher_true_sim_state)
            del self.timer_true_sim_state
            del self.publisher_true_sim_state

        # delete config strings
        del self.timer_true_sim_state_setting
        del self.pub_true_sim_state_setting

        # delete other properties
        del self.shared_ctrl
        del self.lock
        del self.sim_process
        del self.n_u

        return TransitionCallbackReturn.SUCCESS

    @abstractmethod
    def publish_true_sim_state(self) -> osm.TrueSimState:
        """Publish the TrueSimState of the simulator.

        This is the timer callback that publishes the TrueSimState and is expected to call self.publisher_true_sim_state
        internally. Note that the true sim state message is still returned afterwards, mostly for logging/debugging
        purposes. The publish call is the important part, NOT the returned value, since the topic is what the
        ObeliskEstimator subscribes to.

        Returns:
            obelisk_true_sim_state_msg: An Obelisk message type containing the true sim state.
        """

    @abstractmethod
    def run_simulator(self) -> None:
        """Run the simulator.

        The control input into the simulator is accessed through the shared control array. If any simulator
        configuration needs to occur, then the on_configure function should be overridden (and this implementation
        should be called with super()). This function should run the simulator loop and update the state of the
        simulator as long as the node is active.
        """
