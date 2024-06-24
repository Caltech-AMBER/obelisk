import obelisk_control_msgs.msg as ocm
import obelisk_estimator_msgs.msg as oem
import pytest
import rclpy
import rclpy.exceptions

from obelisk_py.control import ObeliskController
from obelisk_py.obelisk_typing import ObeliskControlMsg, ObeliskEstimatorMsg, ObeliskMsg, is_in_bound


class TestObeliskController(ObeliskController):
    """Test ObeliskController class."""

    def update_x_hat(self, x_hat_msg: ObeliskEstimatorMsg) -> None:
        """Update the state estimate."""
        self.x_hat = 1  # [NOTE] dummy implementation

    def compute_control(self) -> ObeliskControlMsg:
        """Compute the control signal."""
        obk_ctrl_msg = ocm.PositionSetpoint()  # [NOTE] dummy implementation
        return obk_ctrl_msg


def test_obelisk_controller() -> None:
    """Test the ObeliskController class."""
    rclpy.init()
    test_controller = TestObeliskController("test_controller")
    parameter_names = [
        "timer_ctrl_config_str",
        "pub_ctrl_config_str",
        "sub_est_config_str",
    ]

    # check that accessing parameters without initializing them raises an error
    with pytest.raises(rclpy.exceptions.ParameterUninitializedException):
        test_controller.get_parameters(parameter_names)

    # set parameters
    test_controller.set_parameters(
        [
            rclpy.parameter.Parameter(
                "timer_ctrl_config_str",
                rclpy.Parameter.Type.STRING,
                ("timer_period_sec:0.001," "callback:compute_control," "callback_group:None"),
            ),
            rclpy.parameter.Parameter(
                "pub_ctrl_config_str",
                rclpy.Parameter.Type.STRING,
                (
                    "msg_type:PositionSetpoint,"
                    "topic:/obelisk/test_controller/control,"
                    "history_depth:10,"
                    "callback_group:None,"
                    "non_obelisk:False"
                ),
            ),
            rclpy.parameter.Parameter(
                "sub_est_config_str",
                rclpy.Parameter.Type.STRING,
                (
                    "msg_type:EstimatedState,"
                    "topic:/obelisk/test_controller/state_estimate,"
                    "callback:update_x_hat,"
                    "history_depth:10,"
                    "callback_group:None,"
                    "non_obelisk:False"
                ),
            ),
        ]
    )

    # check that accessing parameters after setting them works
    try:
        test_controller.get_parameters(parameter_names)
    except rclpy.exceptions.ParameterUninitializedException as e:
        pytest.fail(f"Failed to access parameters after setting them: {e}")

    # check that before calling on_configure, the parameter attributes in the node are not set
    for name in parameter_names:
        assert not hasattr(test_controller, name)

    # also check that the publisher, timer, and subscriber are not set
    assert not hasattr(test_controller, "publisher_ctrl")
    assert not hasattr(test_controller, "timer_ctrl")
    assert not hasattr(test_controller, "subscriber_est")

    # check that calling on_configure sets attributes in the node
    test_controller.on_configure(test_controller._state_machine.current_state)
    for name in parameter_names:
        assert hasattr(test_controller, name)

    assert hasattr(test_controller, "publisher_ctrl")
    assert hasattr(test_controller, "timer_ctrl")
    assert hasattr(test_controller, "subscriber_est")

    # check that on_cleanup resets the attributes in the node
    # TODO(ahl): how do we test that the publisher, timer, and subscriber are destroyed?
    test_controller.on_cleanup(test_controller._state_machine.current_state)
    for name in parameter_names:
        assert not hasattr(test_controller, name)

    # check that we can configure the node again then update the state estimate and compute the control signal
    test_controller.on_configure(test_controller._state_machine.current_state)

    estimated_state_msg = oem.EstimatedState()  # [NOTE] dummy message
    test_controller.update_x_hat(estimated_state_msg)
    assert test_controller.x_hat == 1

    obk_ctrl_msg = test_controller.compute_control()
    assert isinstance(obk_ctrl_msg, ocm.PositionSetpoint)
    assert is_in_bound(type(obk_ctrl_msg), ObeliskControlMsg)
    assert is_in_bound(type(obk_ctrl_msg), ObeliskMsg)

    # destroy node and shutdown rclpy session
    test_controller.destroy_node()
    rclpy.shutdown()
