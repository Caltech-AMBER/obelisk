import obelisk_estimator_msgs.msg as oem
import obelisk_sensor_msgs.msg as osm
import pytest
import rclpy
import rclpy.exceptions

from obelisk_py.estimation import ObeliskEstimator
from obelisk_py.obelisk_typing import ObeliskEstimatorMsg, ObeliskMsg, is_in_bound


class TestObeliskEstimator(ObeliskEstimator):
    """Test ObeliskEstimator class."""

    def sensor_callback1(self, msg: ObeliskMsg) -> None:
        """Sensor callback 1."""
        pass

    def sensor_callback2(self, msg: ObeliskMsg) -> None:
        """Sensor callback 2."""
        pass

    def compute_state_estimate(self) -> ObeliskEstimatorMsg:
        """Compute the state estimate."""
        obk_est_msg = oem.EstimatedState()  # [NOTE] dummy implementation
        return obk_est_msg


def test_obelisk_estimator() -> None:
    """Test the ObeliskEstimator class."""
    rclpy.init()
    test_estimator = TestObeliskEstimator("test_estimator")
    parameter_names = [
        "callback_group_config_strs",
        "timer_est_config_str",
        "pub_est_config_str",
        "sub_sensor_config_strs",
    ]

    # check that accessing parameters without initializing them raises an error
    with pytest.raises(rclpy.exceptions.ParameterUninitializedException):
        test_estimator.get_parameters(parameter_names)

    # set parameters
    test_estimator.set_parameters(
        [
            rclpy.parameter.Parameter(
                "callback_group_config_strs",
                rclpy.Parameter.Type.STRING_ARRAY,
                ["test_cbg:ReentrantCallbackGroup"],
            ),
            rclpy.parameter.Parameter(
                "timer_est_config_str",
                rclpy.Parameter.Type.STRING,
                "timer_period_sec:0.001,callback:compute_state_estimate,callback_group:None",
            ),
            rclpy.parameter.Parameter(
                "pub_est_config_str",
                rclpy.Parameter.Type.STRING,
                (
                    "msg_type:EstimatedState,"
                    "topic:/obelisk/test_estimator/state_estimate,"
                    "history_depth:10,"
                    "callback_group:None,"
                    "non_obelisk:False"
                ),
            ),
            rclpy.parameter.Parameter(
                "sub_sensor_config_strs",
                rclpy.Parameter.Type.STRING_ARRAY,
                [
                    (
                        "msg_type:JointEncoder,"
                        "topic:/obelisk/test_estimator/sensor1,"
                        "callback:sensor_callback1,"
                        "history_depth:10,"
                        "callback_group:None,"
                        "non_obelisk:False"
                    ),
                    (
                        "msg_type:JointEncoder,"
                        "topic:/obelisk/test_estimator/sensor2,"
                        "callback:sensor_callback2,"
                        "history_depth:10,"
                        "callback_group:None,"
                        "non_obelisk:False"
                    ),
                ],
            ),
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

    # also check that the publisher, timer, and subscriber list are not set
    assert not hasattr(test_estimator, "publisher_est")
    assert not hasattr(test_estimator, "timer_est")
    assert not hasattr(test_estimator, "subscriber_sensors")

    # check that calling on_configure sets attributes in the node
    test_estimator.on_configure(test_estimator._state_machine.current_state)
    for name in parameter_names:
        assert hasattr(test_estimator, name)

    assert hasattr(test_estimator, "test_cbg")
    assert hasattr(test_estimator, "publisher_est")
    assert hasattr(test_estimator, "timer_est")
    assert hasattr(test_estimator, "subscriber_sensors")

    # check that on_cleanup resets the attributes in the node
    test_estimator.on_cleanup(test_estimator._state_machine.current_state)
    for name in parameter_names:
        assert not hasattr(test_estimator, name)

    assert not hasattr(test_estimator, "test_cbg")
    assert not hasattr(test_estimator, "publisher_est")
    assert not hasattr(test_estimator, "timer_est")
    assert not hasattr(test_estimator, "subscriber_sensors")

    # check that we can configure the node again and compute the state estimate
    test_estimator.on_configure(test_estimator._state_machine.current_state)

    obk_estimator_msg = test_estimator.compute_state_estimate()
    assert isinstance(obk_estimator_msg, oem.EstimatedState)
    assert is_in_bound(type(obk_estimator_msg), ObeliskEstimatorMsg)
    assert is_in_bound(type(obk_estimator_msg), ObeliskMsg)

    # verify that we can call the sensor callbacks
    obk_sensor1_msg = osm.JointEncoder()  # [NOTE] dummy message
    try:
        test_estimator.sensor_callback1(obk_sensor1_msg)
    except AttributeError:
        pytest.fail("Failed to call sensor_callback1")

    obk_sensor2_msg = osm.JointEncoder()  # [NOTE] dummy message
    try:
        test_estimator.sensor_callback2(obk_sensor2_msg)
    except AttributeError:
        pytest.fail("Failed to call sensor_callback2")

    # destroy node and shutdown rclpy session
    test_estimator.destroy_node()
    rclpy.shutdown()
