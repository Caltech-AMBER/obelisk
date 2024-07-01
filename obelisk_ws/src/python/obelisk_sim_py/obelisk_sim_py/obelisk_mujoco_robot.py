from typing import List, Optional

from rclpy.executors import MultiThreadedExecutor

from obelisk_py.core.utils.ros import spin_obelisk
from obelisk_py.zoo.robot.sim.mujoco import ObeliskMujocoRobot


def main(args: Optional[List] = None) -> None:
    """Main entrypoint."""
    spin_obelisk(args, ObeliskMujocoRobot, MultiThreadedExecutor)


if __name__ == "__main__":
    main()
