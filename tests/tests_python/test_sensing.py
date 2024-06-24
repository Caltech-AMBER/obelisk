import obelisk_sensor_msgs.msg as osm
import pytest
import rclpy
import rclpy.exceptions

from obelisk_py.obelisk_typing import ObeliskMsg, ObeliskSensorMsg, is_in_bound
from obelisk_py.sensing import ObeliskSensor


class TestObeliskSensor(ObeliskSensor):
    """Test ObeliskSensor class."""

    def publish_measurement1(self) -> ObeliskSensorMsg:
        """Publish measurement1 from the sensor."""
        obk_sensor_msg = osm.JointEncoder()  # [NOTE] dummy implementation
        return obk_sensor_msg

    def publish_measurement2(self) -> ObeliskSensorMsg:
        """Publish measurement2 from the sensor."""
        obk_sensor_msg = osm.JointEncoder()  # [NOTE] dummy implementation
        return obk_sensor_msg


def test_obelisk_sensor() -> None:
    """Test the ObeliskSensorMsg class."""
    rclpy.init()
    test_sensor = TestObeliskSensor("test_estimator")
    parameter_names = ["pub_sensor_config_strs"]

    # check that accessing parameters without initializing them raises an error
    with pytest.raises(rclpy.exceptions.ParameterUninitializedException):
        test_sensor.get_parameters(parameter_names)

    # set parameters
    test_sensor.set_parameters(
        [
            rclpy.parameter.Parameter(
                "callback_group_config_strs",
                rclpy.Parameter.Type.STRING_ARRAY,
                ["test_cbg:ReentrantCallbackGroup"],
            ),
            rclpy.parameter.Parameter(
                "pub_sensor_config_strs",
                rclpy.Parameter.Type.STRING_ARRAY,
                [
                    (
                        "msg_type:JointEncoder,"
                        "topic:/obelisk/test_estimator/sensor1,"
                        "history_depth:10,"
                        "callback_group:None,"
                        "non_obelisk:False"
                    ),
                    (
                        "msg_type:JointEncoder,"
                        "topic:/obelisk/test_estimator/sensor2,"
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
        test_sensor.get_parameters(parameter_names)
    except rclpy.exceptions.ParameterUninitializedException as e:
        pytest.fail(f"Failed to access parameters after setting them: {e}")

    # check that before calling on_configure, the parameter attributes in the node are not set
    for name in parameter_names:
        assert not hasattr(test_sensor, name)

    # also check that the publisher, timer, and subscriber list are not set
    assert not hasattr(test_sensor, "test_cbg")
    assert not hasattr(test_sensor, "publisher_sensors")

    # check that calling on_configure sets attributes in the node
    test_sensor.on_configure(test_sensor._state_machine.current_state)
    for name in parameter_names:
        assert hasattr(test_sensor, name)

    assert hasattr(test_sensor, "test_cbg")
    assert hasattr(test_sensor, "publisher_sensors")

    # check that on_cleanup resets the attributes in the node
    test_sensor.on_cleanup(test_sensor._state_machine.current_state)
    for name in parameter_names:
        assert not hasattr(test_sensor, name)

    assert not hasattr(test_sensor, "test_cbg")
    assert not hasattr(test_sensor, "publisher_sensors")

    # check that we can configure the node again and compute a measurement by calling the publishers
    test_sensor.on_configure(test_sensor._state_machine.current_state)

    obk_sensor_msg1 = test_sensor.publish_measurement1()
    obk_sensor_msg2 = test_sensor.publish_measurement2()
    assert isinstance(obk_sensor_msg1, osm.JointEncoder)
    assert is_in_bound(type(obk_sensor_msg1), ObeliskSensorMsg)
    assert is_in_bound(type(obk_sensor_msg1), ObeliskMsg)
    assert isinstance(obk_sensor_msg2, osm.JointEncoder)
    assert is_in_bound(type(obk_sensor_msg2), ObeliskSensorMsg)
    assert is_in_bound(type(obk_sensor_msg2), ObeliskMsg)

    # destroy node and shutdown rclpy session
    test_sensor.destroy_node()
    rclpy.shutdown()
