from typing import List, Optional

from obelisk_control_msgs.msg import PositionSetpoint
from rclpy.executors import MultiThreadedExecutor

from obelisk_py.core.sim.mujoco import ObeliskMujocoRobot
from obelisk_py.core.utils.ros import spin_obelisk


def main(args: Optional[List] = None) -> None:
    """Main entrypoint."""
    ctrl_msg_type = PositionSetpoint
    spin_obelisk(args, ObeliskMujocoRobot, MultiThreadedExecutor, {"ctrl_msg_type": ctrl_msg_type})


if __name__ == "__main__":
    main()
