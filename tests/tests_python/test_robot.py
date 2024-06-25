import obelisk_control_msgs.msg as ocm
import obelisk_sensor_msgs.msg as osm
import pytest
import rclpy
import rclpy.exceptions

from obelisk_py.obelisk_typing import ObeliskControlMsg, ObeliskMsg, ObeliskSensorMsg, is_in_bound
from obelisk_py.robot import ObeliskRobot


class TestObeliskRobot(ObeliskRobot):
    """Test ObeliskRobot class."""

    def apply_control(self, _: ObeliskControlMsg) -> None:
        """Apply the control message to the robot."""
        pass

    def publish_measurement1(self) -> ObeliskSensorMsg:
        """Publish measurement1 from the sensor."""
        obk_sensor_msg = osm.JointEncoder()  # [NOTE] dummy implementation
        return obk_sensor_msg

    def publish_measurement2(self) -> ObeliskSensorMsg:
        """Publish measurement2 from the sensor."""
        obk_sensor_msg = osm.JointEncoder()  # [NOTE] dummy implementation
        return obk_sensor_msg


def test_obelisk_robot() -> None:
    """Test the ObeliskRobot class."""
    rclpy.init()
    test_robot = TestObeliskRobot("test_robot")
    parameter_names = ["sub_ctrl_config_str", "pub_sensor_config_strs"]

    # check that accessing parameters without initializing them raises an error
    with pytest.raises(rclpy.exceptions.ParameterUninitializedException):
        test_robot.get_parameters(parameter_names)

    # set parameters
    test_robot.set_parameters(
        [
            rclpy.parameter.Parameter(
                "callback_group_config_strs",
                rclpy.Parameter.Type.STRING_ARRAY,
                ["test_cbg:ReentrantCallbackGroup"],
            ),
            rclpy.parameter.Parameter(
                "sub_ctrl_config_str",
                rclpy.Parameter.Type.STRING,
                (
                    "msg_type:PositionSetpoint,"
                    "topic:/obelisk/test_robot/ctrl,"
                    "callback:apply_control,"
                    "history_depth:10,"
                    "callback_group:None,"
                    "non_obelisk:False"
                ),
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
        test_robot.get_parameters(parameter_names)
    except rclpy.exceptions.ParameterUninitializedException as e:
        pytest.fail(f"Failed to access parameters after setting them: {e}")

    # check that before calling on_configure, the parameter attributes in the node are not set
    for name in parameter_names:
        assert not hasattr(test_robot, name)

    # also check that the publisher, timer, and subscriber list are not set
    assert not hasattr(test_robot, "test_cbg")
    assert not hasattr(test_robot, "subscriber_ctrl")
    assert not hasattr(test_robot, "publisher_sensors")

    # check that calling on_configure sets attributes in the node
    test_robot.on_configure(test_robot._state_machine.current_state)
    for name in parameter_names:
        assert hasattr(test_robot, name)

    assert hasattr(test_robot, "test_cbg")
    assert hasattr(test_robot, "subscriber_ctrl")
    assert hasattr(test_robot, "publisher_sensors")

    # check that on_cleanup resets the attributes in the node
    test_robot.on_cleanup(test_robot._state_machine.current_state)
    for name in parameter_names:
        assert not hasattr(test_robot, name)

    assert not hasattr(test_robot, "test_cbg")
    assert not hasattr(test_robot, "subscriber_ctrl")
    assert not hasattr(test_robot, "publisher_sensors")

    # check that we can configure the node again and compute a measurement by calling the publishers + apply control
    test_robot.on_configure(test_robot._state_machine.current_state)

    obk_sensor_msg1 = test_robot.publish_measurement1()
    obk_sensor_msg2 = test_robot.publish_measurement2()
    assert isinstance(obk_sensor_msg1, osm.JointEncoder)
    assert is_in_bound(type(obk_sensor_msg1), ObeliskSensorMsg)
    assert is_in_bound(type(obk_sensor_msg1), ObeliskMsg)
    assert isinstance(obk_sensor_msg2, osm.JointEncoder)
    assert is_in_bound(type(obk_sensor_msg2), ObeliskSensorMsg)
    assert is_in_bound(type(obk_sensor_msg2), ObeliskMsg)

    test_robot.apply_control(ocm.PositionSetpoint())

    # destroy node and shutdown rclpy session
    test_robot.destroy_node()
    rclpy.shutdown()
