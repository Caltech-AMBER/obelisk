import ctypes
import multiprocessing
import os
from typing import Callable, List, Optional

import mujoco
import mujoco.viewer
import numpy as np
import obelisk_sensor_msgs.msg as osm
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleState, TransitionCallbackReturn

from obelisk_py.obelisk_typing import ObeliskControlMsg, ObeliskSensorMsg
from obelisk_py.robot import ObeliskSimRobot


class ObeliskMujocoRobot(ObeliskSimRobot):
    """Simulator that runs Mujoco."""

    def __init__(self) -> None:
        """Initialize the mujoco simulator."""
        super().__init__("obelisk_mujoco_robot")
        self.declare_parameter("mujoco_setting", rclpy.Parameter.Type.STRING)

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
        if hasattr(self, "lock"):
            with self.lock:
                self.shared_ctrl[:] = [ctypes.c_double(value) for value in ctrl]

    def _get_msg_type_from_mj_sensor_type(self, sensor_type: str) -> ObeliskSensorMsg:
        """Get the message type from the Mujoco sensor type.

        The supported sensor types are taken as a subset of the ones listed in the mujoco docs:
        https://mujoco.readthedocs.io/en/latest/XMLreference.html#sensor

        Each sensor type should be associated with an already-existing Obelisk sensor message. If one doesn't exist, we
        simply don't support the mujoco sensor type yet.

        Parameters:
            sensor_type: The Mujoco sensor type.

        Returns:
            The Obelisk sensor message type associated with the Mujoco sensor type.
        """
        if sensor_type == "jointpos":
            return osm.JointEncoders
        else:
            raise NotImplementedError(f"Sensor type {sensor_type} not supported! Check your spelling or open a PR.")

    def _create_timer_callback_from_msg_type(
        self,
        msg_type: ObeliskSensorMsg,
        sensor_names: List[str],
        pub_sensor: rclpy.publisher.Publisher,
    ) -> Callable:
        """Create a timer callback from the ObeliskSensorMsg type.

        Parameters:
            msg_type: The Obelisk sensor message type.
            sensor_names: The names of the sensors to read from.
            pub_sensor: The publisher to publish the sensor message to.

        Returns:
            The timer callback function.
        """

        def timer_callback() -> None:
            """Timer callback for the sensor."""
            msg = msg_type()
            y = []
            for sensor_name in sensor_names:
                y.append(self.mj_data.sensor(sensor_name).data)
            msg.y = list(np.concatenate(y))  # like we assume all ObeliskControlMsg objs have a u field, sensors have y
            pub_sensor.publish(msg)

        return timer_callback

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:  # noqa: PLR0915
        """Configure the simulator."""
        super().on_configure(state)
        try:
            self.mujoco_setting = self.get_parameter("mujoco_setting").get_parameter_value().string_value
        except Exception as e:
            self.get_logger().error(f"Could not get the mujoco setting parameter!\n{e}")
            return TransitionCallbackReturn.ERROR

        # parse and check the configuration string
        field_names, value_names = ObeliskMujocoRobot._parse_config_str(self.mujoco_setting)

        required_field_names = ["model_xml_path", "n_u"]
        optional_field_names = ["time_step", "num_steps_per_viz", "sensor_settings"]
        ObeliskMujocoRobot._check_fields(field_names, required_field_names, optional_field_names)
        config_dict = dict(zip(field_names, value_names))

        # load mujoco model
        self.model_xml_path = config_dict["model_xml_path"]
        if not os.path.exists(self.model_xml_path):
            if "OBELISK_ROOT" not in os.environ:
                raise ValueError("OBELISK_ROOT environment variable not set. Run the dev_setup.sh script!")
            model_xml_path = os.path.join(os.environ["OBELISK_ROOT"], "models", self.model_xml_path)
        else:
            model_xml_path = self.model_xml_path
        self.mj_model = mujoco.MjModel.from_xml_path(model_xml_path)
        self.mj_data = mujoco.MjData(self.mj_model)

        # set the configuration parameters for non-sensor fields
        self.n_u = int(config_dict["n_u"])
        assert self.n_u > 0, "Control input dimension must be positive!"
        if "time_step" in config_dict:
            self.time_step = float(config_dict["time_step"])
        else:
            self.time_step = 0.002
        self.mj_model.opt.timestep = self.time_step  # set the timestep of the model to match

        if "num_steps_per_viz" in config_dict:
            self.num_steps_per_viz = int(config_dict["num_steps_per_viz"])
        else:
            self.num_steps_per_viz = 5

        # set the configuration parameters for sensor fields
        if "sensor_settings" in config_dict:
            sensor_setting_dict = {}

            # we must strip the brackets and braces out of the string for parsing reasons
            stripped_sensor_settings = config_dict["sensor_settings"]
            stripped_sensor_settings = stripped_sensor_settings.replace("[", "").replace("]", "")
            stripped_sensor_settings = stripped_sensor_settings.replace("{", "").replace("}", "")

            # the internal delimiter between different sensor group settings is a plus sign
            self.sensor_timers = []
            for sensor_setting in stripped_sensor_settings.split("+"):
                # individual settings are separated by pipes
                sensor_setting_dict = dict([setting.split("=") for setting in sensor_setting.split("|")])

                assert "topic" in sensor_setting_dict and isinstance(sensor_setting_dict["topic"], str)
                assert "dt" in sensor_setting_dict and isinstance(sensor_setting_dict["dt"], str)
                assert "sensor_type" in sensor_setting_dict and isinstance(sensor_setting_dict["sensor_type"], str)
                assert "sensor_names" in sensor_setting_dict and isinstance(sensor_setting_dict["sensor_names"], str)
                topic = sensor_setting_dict["topic"]
                dt = float(sensor_setting_dict["dt"])
                sensor_type = sensor_setting_dict["sensor_type"]
                sensor_names = sensor_setting_dict["sensor_names"].split("&")

                # make sensor pub/timer pair
                msg_type = self._get_msg_type_from_mj_sensor_type(sensor_type)
                cbg = ReentrantCallbackGroup()
                pub_sensor = self.create_publisher(
                    msg_type=msg_type,
                    topic=topic,
                    qos_profile=10,
                    callback_group=cbg,
                )
                timer_callback = self._create_timer_callback_from_msg_type(msg_type, sensor_names, pub_sensor)
                timer_sensor = self.create_timer(
                    timer_period_sec=dt,
                    callback=timer_callback,
                    callback_group=cbg,
                )
                timer_sensor.cancel()
                self.sensor_timers.append(timer_sensor)
        else:
            self.sensor_timers = None

        # setting up the simulator
        self.t_last = multiprocessing.Value("d", 0.0)  # shared mem for t_last
        self.shared_ctrl = multiprocessing.Array(ctypes.c_double, self.n_u, lock=True)
        self.lock = self.shared_ctrl.get_lock()
        self.sim_process = multiprocessing.Process(target=self.run_simulator)
        self.is_viewer_running = multiprocessing.Value("b", True)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate the simulator."""
        super().on_activate(state)
        self.t_last.value = self.t
        if self.sensor_timers is not None:
            for timer in self.sensor_timers:
                timer.reset()
        self.sim_process.start()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Deactivate the simulator."""
        super().on_deactivate(state)
        if self.timers is not None:
            for timer in self.timers:
                timer.cancel()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Clean up the simulator."""
        super().on_cleanup(state)
        # terminate the simulation process
        if self.sim_process.is_alive():
            self.is_viewer_running.value = False
            self.sim_process.terminate()
            self.sim_process.join()
        return TransitionCallbackReturn.SUCCESS

    def apply_control(self, control_msg: ObeliskControlMsg) -> None:
        """Apply the control message.

        We assume that the control message is a vector of control inputs and is fully compatible with the data.ctrl
        field of a mujoco model. YOU MUST CHECK THIS YOURSELF!
        """
        self._set_shared_ctrl(control_msg.u)

    def publish_true_sim_state(self) -> osm.TrueSimState:
        """Publish the true simulator state."""
        return osm.TrueSimState()  # TODO(ahl): no-op, haven't decided on what goes in here yet until later PR

    def run_simulator(self) -> None:
        """Run the mujoco simulator."""
        with mujoco.viewer.launch_passive(self.mj_model, self.mj_data) as viewer:
            while viewer.is_running() and self.is_viewer_running.value:
                # simulate at realtime rate
                # TODO(ahl): allow non-realtime rates
                t = self.t
                t_last = self.t_last.value
                dt = t - t_last
                if dt < self.num_steps_per_viz * self.time_step:
                    continue

                for _ in range(self.num_steps_per_viz):
                    self.mj_data.ctrl[:] = np.array(self.shared_ctrl)
                    mujoco.mj_step(self.mj_model, self.mj_data)
                viewer.sync()
                self.t_last.value = t


def main(args: Optional[List] = None) -> None:
    """Main entrypoint."""
    rclpy.init(args=args)
    obelisk_mujoco_robot = ObeliskMujocoRobot()
    executor = MultiThreadedExecutor()
    executor.add_node(obelisk_mujoco_robot)
    executor.spin()
    obelisk_mujoco_robot.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
