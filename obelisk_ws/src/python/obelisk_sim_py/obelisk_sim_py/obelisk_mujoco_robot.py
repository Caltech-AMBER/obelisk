from typing import List, Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor

from obelisk_py.zoo.robot.sim.mujoco import ObeliskMujocoRobot


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
