from typing import List, Optional

import mujoco
import mujoco.viewer
import numpy as np
import obelisk_sensor_msgs.msg as osm
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleState, TransitionCallbackReturn

from obelisk_py.obelisk_typing import ObeliskControlMsg
from obelisk_py.robot import ObeliskSimRobot


class ObeliskMujocoRobot(ObeliskSimRobot):
    """Simulator that runs Mujoco."""

    def __init__(self) -> None:
        """Initialize the mujoco simulator."""
        super().__init__("obelisk_mujoco_robot")
        self.declare_parameter("mujoco_setting", rclpy.Parameter.Type.STRING)

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the simulator."""
        super().on_configure(state)
        self.mujoco_setting = self.get_parameter("mujoco_setting").get_parameter_value().string_value

        # parse and check the configuration string
        field_names, value_names = ObeliskMujocoRobot._parse_config_str(self.mujoco_setting)
        required_field_names = ["model_xml_path"]
        optional_field_names = ["dt_phys", "num_phys_steps_per_viz"]
        ObeliskMujocoRobot._check_fields(field_names, required_field_names, optional_field_names)
        config_dict = dict(zip(field_names, value_names))

        # set the configuration parameters
        self.model_xml_path = config_dict["model_xml_path"]
        if "dt_phys" in config_dict:
            self.dt_phys = float(config_dict["dt_phys"])
        else:
            self.dt_phys = 0.002

        if "num_phys_steps_per_viz" in config_dict:
            self.num_phys_steps_per_viz = int(config_dict["num_phys_steps_per_viz"])
        else:
            self.num_phys_steps_per_viz = 5

        # load mujoco model
        self.mj_model = mujoco.MjModel.from_xml_path(self.model_xml_path)
        self.mj_data = mujoco.MjData(self.mj_model)
        self.pause = False
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate the simulator."""
        super().on_activate(state)
        self.t_last = self.t
        self.pause = False
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Deactivate the simulator."""
        super().on_deactivate(state)
        self.pause = True
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
        with mujoco.viewer.launch_passive(self.m, self.d) as viewer:
            while viewer.is_running():
                # simulate at realtime rate
                # TODO(ahl): allow non-realtime rates
                t = self.t
                dt = t - self.t_last
                if dt < self.num_phys_steps_per_viz * self.dt_phys:
                    continue

                for _ in range(self.num_phys_steps_per_viz):
                    self.d.ctrl[:] = np.array(self.shared_ctrl)
                    mujoco.mj_step(self.m, self.d)
                viewer.sync()
                self.t_last = t


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
