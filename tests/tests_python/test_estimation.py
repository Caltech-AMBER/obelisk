import obelisk_estimator_msgs.msg as oem
import pytest
import rclpy
import rclpy.exceptions

from obelisk_py.estimation import ObeliskEstimator
from obelisk_py.obelisk_typing import ObeliskEstimatorMsg, ObeliskMsg, is_in_bound


class TestObeliskEstimator(ObeliskEstimator):
    """Test ObeliskEstimator class."""

    def compute_state_estimate(self) -> ObeliskEstimatorMsg:
        """Compute the state estimate."""
        obk_est_msg = oem.EstimatedState()  # [NOTE] dummy implementation
        return obk_est_msg


def test_obelisk_estimator() -> None:
    """Test the ObeliskEstimator class."""
    rclpy.init()
    test_estimator = TestObeliskEstimator("test_estimator")
    parameter_names = [
        "dt_est",
        "msg_type_est",
        "history_depth_est",
        "cb_group_est",
    ]

    # check that accessing parameters without initializing them raises an error
    with pytest.raises(rclpy.exceptions.ParameterUninitializedException):
        test_estimator.get_parameters(parameter_names)

    # set parameters
    test_estimator.set_parameters(
        [
            rclpy.parameter.Parameter("dt_est", rclpy.Parameter.Type.DOUBLE, 0.1),
            rclpy.parameter.Parameter("msg_type_est", rclpy.Parameter.Type.STRING, "EstimatedState"),
            rclpy.parameter.Parameter("history_depth_est", rclpy.Parameter.Type.INTEGER, 10),
            rclpy.parameter.Parameter("cb_group_est", rclpy.Parameter.Type.STRING, "None"),
        ]
    )

    # check that accessing parameters after setting them works
    try:
        test_estimator.get_parameters(parameter_names)
    except rclpy.exceptions.ParameterUninitializedException as e:
        pytest.fail(f"Failed to access parameters after setting them: {e}")

    # check that before calling on_configure, the parameter attributes in the node are not set
    for name in parameter_names:
        assert not hasattr(test_estimator, name)

    # also check that the publisher, timer, and subscriber are not set
    assert not hasattr(test_estimator, "publisher_est")
    assert not hasattr(test_estimator, "timer_est")

    # check that calling on_configure sets attributes in the node
    test_estimator.on_configure(test_estimator._state_machine.current_state)
    for name in parameter_names:
        assert hasattr(test_estimator, name)

    assert hasattr(test_estimator, "publisher_est")
    assert hasattr(test_estimator, "timer_est")

    # check that on_cleanup resets the attributes in the node
    # TODO(ahl): how do we test that the publisher and timer are destroyed?
    test_estimator.on_cleanup(test_estimator._state_machine.current_state)
    for name in parameter_names:
        assert not hasattr(test_estimator, name)

    # check that we can configure the node again and compute the state estimate
    test_estimator.on_configure(test_estimator._state_machine.current_state)

    obk_estimator_msg = test_estimator.compute_state_estimate()
    assert isinstance(obk_estimator_msg, oem.EstimatedState)
    assert is_in_bound(type(obk_estimator_msg), ObeliskEstimatorMsg)
    assert is_in_bound(type(obk_estimator_msg), ObeliskMsg)

    # destroy node and shutdown rclpy session
    test_estimator.destroy_node()
    rclpy.shutdown()
