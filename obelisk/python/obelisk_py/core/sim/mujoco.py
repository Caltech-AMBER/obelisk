import ctypes
import multiprocessing
import os
from typing import Callable, List, Tuple, Type

import mujoco
import mujoco.viewer
import numpy as np
import obelisk_sensor_msgs.msg as osm
import rclpy
from ament_index_python.packages import get_package_share_directory
from mujoco import MjData, MjModel, mj_forward, mj_name2id, mj_step  # type: ignore
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.lifecycle import LifecycleState, TransitionCallbackReturn
from rclpy.publisher import Publisher

from obelisk_py.core.obelisk_typing import ObeliskControlMsg, ObeliskSensorMsg, is_in_bound
from obelisk_py.core.robot import ObeliskSimRobot


class ObeliskMujocoRobot(ObeliskSimRobot):
    """Simulator that runs Mujoco."""

    def __init__(self, node_name: str = "obelisk_mujoco_robot") -> None:
        """Initialize the mujoco simulator."""
        super().__init__(node_name)
        self.declare_parameter("mujoco_setting", rclpy.Parameter.Type.STRING)

    def _get_msg_type_from_string(self, msg_type_str: str) -> Type[ObeliskSensorMsg]:
        """Get the message type from a string.

        Parameters:
            msg_type_str: The name of the msg_type.

        Returns:
            The Obelisk sensor message type associated with the message type.
        """
        if msg_type_str == "ObkJointEncoders":
            assert is_in_bound(osm.ObkJointEncoders, ObeliskSensorMsg)
            return osm.ObkJointEncoders  # type: ignore
        elif msg_type_str == "ObkImu":
            assert is_in_bound(osm.ObkImu, ObeliskSensorMsg)
            return osm.ObkImu  # type: ignore
        elif msg_type_str == "ObkFramePose":
            assert is_in_bound(osm.ObkFramePose, ObeliskSensorMsg)
            return osm.ObkFramePose  # type: ignore
        else:
            raise NotImplementedError(f"Message type {msg_type_str} not supported! Check your spelling or open a PR.")

    def _get_time_from_sim(self) -> Tuple[float, float]:
        """Get the time from the simulator used to populate msg header fields.

        Returns:
            sec: The seconds field of the time.
            nsec: The nanoseconds field of the time.
        """
        sec = int(np.floor(self.shared_time.value))
        nsec = int((self.shared_time.value - sec) * 1e9)
        return sec, nsec

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
            # verify that all joints are hinge or slider joints
            for sensor_name in mj_sensor_names:
                sensor_id = mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)  # type: ignore

                joint_id = self.mj_model.sensor_objid[sensor_id]
                joint_name = self.mj_model.joint(joint_id).name
                joint_type = self.mj_model.jnt_type[joint_id]

                if joint_type not in [mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE]:  # type: ignore
                    self.get_logger().error(f"Joint {joint_name} should be a hinge or slide joint!")
                    raise ValueError(f"Joint {joint_name} should be a hinge or slide joint!")

            def timer_callback() -> None:
                """Timer callback for ObkJointEncoders."""
                msg = osm.ObkJointEncoders()
                joint_pos = []
                joint_vel = []
                joint_names = []

                for sensor_name, obk_sensor_field in zip(mj_sensor_names, obk_sensor_fields):
                    # get the sensor id and address
                    sensor_id = mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)  # type: ignore
                    sensor_adr = self.mj_model.sensor_adr[sensor_id]
                    if sensor_id == -1:
                        self.get_logger().error(f"Sensor {sensor_name} not found!")
                        raise ValueError(f"Sensor {sensor_name} not found!")

                    if obk_sensor_field == "joint_pos":
                        joint_pos.append(self.shared_sensordata[sensor_adr])

                        # getting joint name from the joint position sensor
                        joint_id = self.mj_model.sensor_objid[sensor_id]
                        joint_names.append(self.mj_model.joint(joint_id).name)

                    elif obk_sensor_field == "joint_vel":
                        joint_vel.append(self.shared_sensordata[sensor_adr])

                    else:
                        self.get_logger().error(f"Unknown sensor name {sensor_name} for message type {msg_type}!")
                        raise ValueError(f"Unknown sensor name {sensor_name} for message type {msg_type}!")

                # filling out fields
                msg.joint_pos = joint_pos
                msg.joint_vel = joint_vel
                msg.joint_names = joint_names

                # timestamp
                sec, nsec = self._get_time_from_sim()
                msg.stamp.sec = sec
                msg.stamp.nanosec = nsec

                pub_sensor.publish(msg)

        elif msg_type == osm.ObkImu:

            def timer_callback() -> None:
                """Timer callback for ObkImu."""
                msg = osm.ObkImu()
                has_acc, has_gyro, has_framequat = False, False, False
                for sensor_name, obk_sensor_field in zip(mj_sensor_names, obk_sensor_fields):
                    # get the sensor id and address
                    sensor_id = mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)  # type: ignore
                    sensor_adr = self.mj_model.sensor_adr[sensor_id]
                    if sensor_id == -1:
                        self.get_logger().error(f"Sensor {sensor_name} not found!")
                        raise ValueError(f"Sensor {sensor_name} not found!")

                    # [NOTE] We only use the frame associated with the accelerometer. If the gyro or magnetometer is at
                    # a different frame, then it needs to be its own sensor. Be sure the gyro, magnetometer, and
                    # accelerometer are at the same frame.
                    if obk_sensor_field == "accelerometer":
                        if not has_acc:
                            msg.linear_acceleration.x = self.shared_sensordata[sensor_adr + 0]
                            msg.linear_acceleration.y = self.shared_sensordata[sensor_adr + 1]
                            msg.linear_acceleration.z = self.shared_sensordata[sensor_adr + 2]

                            # accelerometers should always be mounted to sites
                            site_id = self.mj_model.sensor_objid[sensor_id]
                            msg.header.frame_id = self.mj_model.site(site_id).name
                            has_acc = True
                        else:
                            self.get_logger().error(f"Multiple accelerometers detected! Ignoring {sensor_name}.")

                    elif obk_sensor_field == "gyro":
                        if not has_gyro:
                            msg.angular_velocity.x = self.shared_sensordata[sensor_adr + 0]
                            msg.angular_velocity.y = self.shared_sensordata[sensor_adr + 1]
                            msg.angular_velocity.z = self.shared_sensordata[sensor_adr + 2]
                            has_gyro = True
                        else:
                            self.get_logger().error(f"Multiple gyroscopes detected! Ignoring {sensor_name}.")

                    elif obk_sensor_field == "framequat":
                        if not has_framequat:
                            msg.orientation.x = self.shared_sensordata[sensor_adr + 0]
                            msg.orientation.y = self.shared_sensordata[sensor_adr + 1]
                            msg.orientation.z = self.shared_sensordata[sensor_adr + 2]
                            msg.orientation.w = self.shared_sensordata[sensor_adr + 3]
                            has_framequat = True
                        else:
                            self.get_logger().error(f"Multiple framequats detected! Ignoring {sensor_name}.")

                    else:
                        self.get_logger().error(f"Unknown sensor name {sensor_name} for message type {msg_type}!")
                        raise ValueError(f"Unknown sensor name {sensor_name} for message type {msg_type}!")

                # timestamp
                sec, nsec = self._get_time_from_sim()
                msg.header.stamp.sec = sec
                msg.header.stamp.nanosec = nsec

                pub_sensor.publish(msg)

        elif msg_type == osm.ObkFramePose:

            def timer_callback() -> None:
                """Timer callback for ObkFramePose."""
                msg = osm.ObkFramePose()

                has_framepos = False
                has_framequat = False

                for sensor_name, obk_sensor_field in zip(mj_sensor_names, obk_sensor_fields):
                    # get the sensor id and address
                    sensor_id = mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)  # type: ignore
                    sensor_adr = self.mj_model.sensor_adr[sensor_id]
                    if sensor_id == -1:
                        self.get_logger().error(f"Sensor {sensor_name} not found!")
                        raise ValueError(f"Sensor {sensor_name} not found!")

                    if obk_sensor_field == "framepos":
                        if not has_framepos:
                            msg.position.x = self.shared_sensordata[sensor_adr + 0]
                            msg.position.y = self.shared_sensordata[sensor_adr + 1]
                            msg.position.z = self.shared_sensordata[sensor_adr + 2]

                            # framepos sensors can be mounted onto different mujoco objects (the frame we measure)
                            obj_type = self.mj_model.sensor_objtype[sensor_id]
                            obj_id = self.mj_model.sensor_objid[sensor_id]
                            if obj_type == mujoco.mjtObj.mjOBJ_SITE:  # type: ignore
                                msg.frame_name = self.mj_model.site(obj_id).name
                                msg.header.frame_id = self.mj_model.site(obj_id).name
                            elif obj_type == mujoco.mjtObj.mjOBJ_BODY:  # type: ignore
                                msg.frame_name = self.mj_model.body(obj_id).name
                            elif obj_type == mujoco.mjtObj.mjOBJ_GEOM:  # type: ignore
                                msg.frame_name = self.mj_model.geom(obj_id).name
                            elif obj_type == mujoco.mjtObj.mjOBJ_CAMERA:  # type: ignore
                                msg.frame_name = self.mj_model.cam(obj_id).name
                            else:
                                err_msg = (
                                    f"Framepos sensor {sensor_name} is not associated with a supported Mujoco "
                                    " object type! Current object type: {obj_type}"
                                )
                                self.get_logger().error(err_msg)
                                raise ValueError(err_msg)

                            # now we also populate the reference frame of the sensor
                            if self.mj_model.sensor_refid[sensor_id] == -1:
                                msg.header.frame_id = "world"  # TODO: consider not hardcoding this
                            else:
                                ref_type = self.mj_model.sensor_reftype[sensor_id]
                                ref_id = self.mj_model.sensor_refid[sensor_id]
                                if ref_type == mujoco.mjtObj.mjOBJ_SITE:  # type: ignore
                                    msg.header.frame_id = self.mj_model.site(ref_id).name
                                elif ref_type == mujoco.mjtObj.mjOBJ_BODY:  # type: ignore
                                    msg.header.frame_id = self.mj_model.body(ref_id).name
                                elif ref_type == mujoco.mjtObj.mjOBJ_GEOM:  # type: ignore
                                    msg.header.frame_id = self.mj_model.geom(ref_id).name
                                elif ref_type == mujoco.mjtObj.mjOBJ_CAMERA:  # type: ignore
                                    msg.header.frame_id = self.mj_model.cam(ref_id).name
                                else:
                                    err_msg = (
                                        f"Framepos sensor {sensor_name} has an unsupported reference object type! "
                                        f"Current object type: {ref_type}"
                                    )
                                    self.get_logger().error(err_msg)
                                    raise ValueError(err_msg)

                            has_framepos = True

                        else:
                            self.get_logger().error(f"Multiple frameposes detected! Ignoring {sensor_name}.")

                    elif obk_sensor_field == "framequat":
                        if not has_framequat:
                            msg.orientation.x = self.shared_sensordata[sensor_adr + 0]
                            msg.orientation.y = self.shared_sensordata[sensor_adr + 1]
                            msg.orientation.z = self.shared_sensordata[sensor_adr + 2]
                            msg.orientation.w = self.shared_sensordata[sensor_adr + 3]
                            has_framequat = True
                        else:
                            self.get_logger().error(f"Multiple framequats detected! Ignoring {sensor_name}.")

                    else:
                        self.get_logger().error(f"Unknown sensor name {sensor_name} for message type {msg_type}!")
                        raise ValueError(f"Unknown sensor name {sensor_name} for message type {msg_type}!")

                # timestamp
                sec, nsec = self._get_time_from_sim()
                msg.header.stamp.sec = sec
                msg.header.stamp.nanosec = nsec

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
        optional_field_names = ["robot_pkg", "time_step", "num_steps_per_viz", "sensor_settings"]
        ObeliskMujocoRobot._check_fields(field_names, required_field_names, optional_field_names)
        config_dict = dict(zip(field_names, value_names))

        # load mujoco model
        self.model_xml_path = config_dict["model_xml_path"]
        self.robot_pkg = config_dict.get("robot_pkg", None)
        assert isinstance(self.model_xml_path, str), "Model XML path must be a string!"
        assert isinstance(self.robot_pkg, str) or self.robot_pkg is None, "Robot package must be a string or None!"
        if not os.path.exists(self.model_xml_path):
            if self.robot_pkg is not None and self.robot_pkg.lower() != "none":
                share_directory = get_package_share_directory(self.robot_pkg)
                model_xml_path = os.path.join(share_directory, "mujoco", self.model_xml_path)
            else:
                self.get_logger().error(
                    "Provided Mujoco XML is NOT an absolute path and robot_pkg is None or not specified. "
                    "Please provide a valid Mujoco XML path."
                )
                return TransitionCallbackReturn.ERROR
        else:
            model_xml_path = self.model_xml_path

        # initialize the mujoco model and data
        try:
            self.mj_model = MjModel.from_xml_path(model_xml_path)
            self.mj_data = MjData(self.mj_model)
        except Exception as e:
            self.get_logger().error(f"Could not load the Mujoco model!\n{e}")
            return TransitionCallbackReturn.ERROR
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

        # setting up the shared memory variables
        self.shared_ctrl = multiprocessing.Array(ctypes.c_double, self.n_u)
        self.shared_sensordata = multiprocessing.Array(ctypes.c_double, len(self.mj_data.sensordata))
        self.shared_time = multiprocessing.Value(ctypes.c_double, 0.0)

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
                msg_type_str = sensor_setting_dict["msg_type"]

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

        return TransitionCallbackReturn.SUCCESS

    def apply_control(self, control_msg: ObeliskControlMsg) -> None:
        """Apply the control message.

        We assume that the control message is a vector of control inputs and is fully compatible with the data.ctrl
        field of a sim model. YOU MUST CHECK THIS YOURSELF!
        """
        self._set_shared_ctrl(control_msg.u_mujoco)

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
                    if hasattr(self, "lock"):
                        with self.lock:
                            self.shared_sensordata[:] = list(self.mj_data.sensordata)
                            self.shared_time.value = self.mj_data.time
                viewer.sync()
                self.t_last.value = t
