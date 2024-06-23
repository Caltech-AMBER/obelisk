import obelisk_control_msgs.msg
import obelisk_estimator_msgs.msg
import rclpy

from obelisk_py.control import ObeliskController
from obelisk_py.obelisk_typing import ObeliskControlMsg, ObeliskEstimatorMsg, ObeliskMsg


class TestObeliskController(ObeliskController):
    """Test ObeliskController class."""

    def update_x_hat(self, x_hat_msg: ObeliskEstimatorMsg) -> None:
        """Update the state estimate."""
        self.x_hat = 1  # [NOTE] dummy implementation

    def compute_control(self) -> ObeliskControlMsg:
        """Compute the control signal."""
        obk_ctrl_msg = obelisk_control_msgs.msg.PositionSetpoint()  # [NOTE] dummy implementation
        return obk_ctrl_msg


def test_obelisk_controller() -> None:
    """Test the ObeliskController class."""
    rclpy.init()
    test_controller = TestObeliskController("test_controller", obelisk_control_msgs.msg.PositionSetpoint)

    # check that the controller is initialized correctly
    assert test_controller.x_hat is None

    # check that the controller updates the state estimate correctly
    estimated_state_msg = obelisk_estimator_msgs.msg.EstimatedState()  # [NOTE] dummy message
    test_controller.update_x_hat(estimated_state_msg)
    assert test_controller.x_hat == 1

    # check that the controller computes the control signal correctly
    obk_ctrl_msg = test_controller.compute_control()
    assert isinstance(obk_ctrl_msg, obelisk_control_msgs.msg.PositionSetpoint)
    assert isinstance(obk_ctrl_msg, ObeliskControlMsg)
    assert isinstance(obk_ctrl_msg, ObeliskMsg)
    test_controller.destroy_node()
