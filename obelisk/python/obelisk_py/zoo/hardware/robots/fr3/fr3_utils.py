import atexit
import time

from franky import Gripper, NetworkException, Robot, RobotWebSession
from franky.robot_web_session import TakeControlTimeoutError


def setup_robot(robot_ip: str, username: str, password: str) -> tuple[Robot, Gripper, RobotWebSession | None]:
    """Sets up the robot and gripper."""
    web_session = RobotWebSession(
        robot_ip,
        username=username,
        password=password,
    )
    try:
        robot = Robot(robot_ip)
        gripper = Gripper(robot_ip)
    except NetworkException:

        def _release_control_on_exit() -> None:
            """Release control of the robot on exit."""
            web_session.release_control()

        atexit.register(_release_control_on_exit)

        web_session.open()
        try:
            web_session.take_control(wait_timeout=1.0, force=False)
        except TakeControlTimeoutError:
            web_session.take_control(wait_timeout=30.0, force=True)
        if not web_session.has_control():
            raise RuntimeError("Failed to take control of the robot!") from None
        print("Successfully taken control of the robot!")
        web_session.unlock_brakes()
        time.sleep(1.0)
        web_session.enable_fci()
        time.sleep(1.0)
        robot = Robot(robot_ip)
        gripper = Gripper(robot_ip)
    print("Robot ready for control!")
    return robot, gripper, web_session
