import ctypes
import multiprocessing
import os
from typing import Callable, List, Type

import mujoco
import mujoco.viewer
import numpy as np
import obelisk_sensor_msgs.msg as osm
import rclpy
from mujoco import MjData, MjModel, mj_forward, mj_name2id, mj_step  # type: ignore
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.lifecycle import LifecycleState, TransitionCallbackReturn
from rclpy.publisher import Publisher

from obelisk_py.core.obelisk_typing import ObeliskSensorMsg, is_in_bound
from obelisk_py.core.robot import ObeliskSimRobot


class ObeliskMujocoRobot(ObeliskSimRobot):
    """Simulator that runs Mujoco."""

    def __init__(self, node_name: str = "obelisk_mujoco_robot") -> None:
        """Initialize the mujoco simulator."""
        super().__init__(node_name)
        self.declare_parameter("mujoco_setting", rclpy.Parameter.Type.STRING)

    def _get_msg_type_from_string(self, msg_type: str) -> Type[ObeliskSensorMsg]:
        """Get the message type from a string.

        Parameters:
            msg_type_str: The name of the msg_type.

        Returns:
            The Obelisk sensor message type associated with the message type.
        """
        if msg_type == "ObkJointEncoders":
            assert is_in_bound(osm.ObkJointEncoders, ObeliskSensorMsg)
            return osm.ObkJointEncoders  # type: ignore
        else:
            raise NotImplementedError(f"Message type {msg_type} not supported! Check your spelling or open a PR.")

    def _create_timer_callback_from_msg_type(
        self,
        msg_type: ObeliskSensorMsg,
        mj_sensor_names: List[str],
        obk_sensor_fields: List[str],
        pub_sensor: Publisher,
    ) -> Callable:
        """Create a timer callback from the ObeliskSensorMsg type.

        We can define this util for the mujoco simulator because we can enumerate all of the mujoco sensor types and
        write custom timer callbacks for each of them. We essentially convert mujoco sensor types to Obelisk sensor
        messages here.

        Parameters:
            msg_type: The Obelisk sensor message type.
            mj_sensor_names: The names of the mujoco sensors to read from, which are defined in the XML.
            obk_sensor_fields: The names of the fields in the Obelisk sensor message matching the msg_type argument.
            pub_sensor: The publisher to publish the sensor message to.

        Returns:
            The timer callback function.
        """
        if msg_type == osm.ObkJointEncoders:

            def _jnt_name_from_sensor_name(sensor_name: str) -> str:
                """Helper function to get the joint name from the sensor name."""
                # getting name
                sensor_id = mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)
                joint_id = self.mj_model.sensor_objid[sensor_id]
                joint_name = self.mj_model.joint(joint_id).name

                # verifying the joint is a revolute or prismatic joint
                joint_type = self.mj_model.jnt_type[joint_id]
                if joint_type not in [mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE]:
                    self.get_logger().error(f"Joint {joint_name} should be a hinge or slide joint!")
                    raise ValueError(f"Joint {joint_name} should be a hinge or slide joint!")

                return joint_name

            # getting the names associated with the sensors and the joints
            mj_joint_pos_names = []
            mj_joint_vel_names = []
            joint_names = []

            for mj_sensor_name, obk_sensor_field in zip(mj_sensor_names, obk_sensor_fields):
                assert self.mj_model.sensor(mj_sensor_name) is not None
                if obk_sensor_field == "jointpos":
                    mj_joint_pos_names.append(mj_sensor_name)
                    joint_names.append(_jnt_name_from_sensor_name(mj_sensor_name))
                elif obk_sensor_field == "jointvel":
                    mj_joint_vel_names.append(mj_sensor_name)
                else:
                    self.get_logger().error(f"Unknown sensor field {obk_sensor_field} for message type {msg_type}!")
                    raise ValueError(f"Unknown sensor field {obk_sensor_field} for message type {msg_type}!")

            def timer_callback() -> None:
                """Timer callback for the sensor."""
                msg = osm.ObkJointEncoders()
                msg.joint_names = joint_names  # once we know this, it doesn't change

                # the actual joint positions and velocities are updated every time the timer callback is called
                joint_pos = []
                joint_vel = []
                for sensor_name in mj_joint_pos_names:
                    joint_pos.append(self.mj_data.sensor(sensor_name).data.item())
                for sensor_name in mj_joint_vel_names:
                    joint_vel.append(self.mj_data.sensor(sensor_name).data.item())
                msg.joint_pos = joint_pos
                msg.joint_vel = joint_vel
                pub_sensor.publish(msg)

        else:
            raise NotImplementedError(f"Message type {msg_type} not supported! Check your spelling or open a PR.")

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
        assert isinstance(self.model_xml_path, str), "Model XML path must be a string!"
        if not os.path.exists(self.model_xml_path):
            if "OBELISK_ROOT" not in os.environ:
                raise ValueError("OBELISK_ROOT environment variable not set. Run the dev_setup.sh script!")
            model_xml_path = os.path.join(os.environ["OBELISK_ROOT"], "models", self.model_xml_path)
        else:
            model_xml_path = self.model_xml_path
        self.mj_model = MjModel.from_xml_path(model_xml_path)
        self.mj_data = MjData(self.mj_model)
        mj_forward(self.mj_model, self.mj_data)

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
            assert isinstance(stripped_sensor_settings, str)
            stripped_sensor_settings = stripped_sensor_settings.replace("[", "").replace("]", "")
            stripped_sensor_settings = stripped_sensor_settings.replace("{", "").replace("}", "")

            # the internal delimiter between different sensor group settings is a plus sign
            for i, sensor_setting in enumerate(stripped_sensor_settings.split("+")):
                # individual settings are separated by pipes
                sensor_setting_dict = dict([setting.split("=") for setting in sensor_setting.split("|")])

                assert "topic" in sensor_setting_dict and isinstance(sensor_setting_dict["topic"], str)
                assert "dt" in sensor_setting_dict and isinstance(sensor_setting_dict["dt"], str)
                assert "msg_type" in sensor_setting_dict and isinstance(sensor_setting_dict["msg_type"], str)
                assert "sensor_names" in sensor_setting_dict and isinstance(sensor_setting_dict["sensor_names"], str)
                topic = sensor_setting_dict["topic"]
                dt = float(sensor_setting_dict["dt"])
                msg_type_str = sensor_setting_dict["sensor_type"]

                # the mujoco sensor names and the correspond Obelisk message field types are delimited by "$"
                sensor_names_and_fields = sensor_setting_dict["sensor_names"].split("&")
                mj_sensor_names = [name_and_type.split("$")[0] for name_and_type in sensor_names_and_fields]
                obk_sensor_fields = [name_and_type.split("$")[1] for name_and_type in sensor_names_and_fields]

                # make sensor pub/timer pair
                msg_type = self._get_msg_type_from_string(msg_type_str)
                cbg = ReentrantCallbackGroup()
                pub_sensor = self.create_publisher(
                    msg_type=msg_type,
                    topic=topic,
                    qos_profile=10,
                    callback_group=cbg,
                )
                timer_callback = self._create_timer_callback_from_msg_type(
                    msg_type,
                    mj_sensor_names,
                    obk_sensor_fields,
                    pub_sensor,
                )
                timer_sensor = self.create_timer(
                    timer_period_sec=dt,
                    callback=timer_callback,
                    callback_group=cbg,
                )
                timer_sensor.cancel()
                self.obk_publishers[f"sensor_group_{i}"] = pub_sensor
                self.obk_timers[f"sensor_group_{i}"] = timer_sensor

        # setting up the shared_ctrl array
        self.shared_ctrl = multiprocessing.Array(ctypes.c_double, self.n_u)

        return TransitionCallbackReturn.SUCCESS

    def publish_true_sim_state(self) -> osm.TrueSimState:
        """Publish the true simulator state."""
        return osm.TrueSimState()  # TODO(ahl): no-op, haven't decided on what goes in here yet until later PR

    def run_simulator(self) -> None:
        """Run the mujoco simulator."""
        with mujoco.viewer.launch_passive(self.mj_model, self.mj_data) as viewer:
            while viewer.is_running() and self.is_sim_running.value:
                # simulate at realtime rate
                # TODO(ahl): allow non-realtime rates
                t = self.t
                t_last = self.t_last.value
                dt = t - t_last
                if dt < self.num_steps_per_viz * self.time_step:
                    continue

                for _ in range(self.num_steps_per_viz):
                    self.mj_data.ctrl[:] = np.array(self.shared_ctrl)
                    mj_step(self.mj_model, self.mj_data)
                viewer.sync()
                self.t_last.value = t
