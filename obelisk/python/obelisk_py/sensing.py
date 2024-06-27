import rclpy
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

from obelisk_py.node import ObeliskNode


class ObeliskSensor(ObeliskNode):
    """Abstract Obelisk sensor node.

    Obelisk sensors interface directly with sensing hardware. This could mean that this node runs from the robot, runs
    from some offboard computer which connects to the sensors, or anything else. ObeliskSensors don't nominally need to
    subscribe to any topics. They simply expect to publish some number of sensor messages.

    This node is not abstract, but it isn't functional. We expect that the end user implements some functionality that
    will call some of the publishers configured by the configuration string.
    """

    def __init__(self, node_name: str) -> None:
        """Initialize the Obelisk sensor."""
        super().__init__(node_name)
        self.declare_parameter("pub_sensor_settings", rclpy.Parameter.Type.STRING_ARRAY)

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the sensor."""
        super().on_configure(state)

        # parsing config strings
        self.pub_sensor_settings = self.get_parameter("pub_sensor_settings").get_parameter_value().string_array_value
        assert (
            self.pub_sensor_settings != [""] and len(self.pub_sensor_settings) > 0
        ), "pub_sensor_settings must be a non-empty list of strings."

        # create publishers
        # TODO(ahl): under this current version of the code, we cannot auto-create the publishers because we don't know
        # the message types in both the python and c++ versions of the code.
        self.publisher_sensors = []
        # for sensor_setting in self.pub_sensor_settings:
        #     pub_sensor = self._create_publisher_from_config_str(sensor_setting)
        #     self.publisher_sensors.append(pub_sensor)

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Clean up the sensor."""
        super().on_cleanup(state)

        # destroy publishers + config strings
        for sensor_publisher in self.publisher_sensors:
            self.destroy_publisher(sensor_publisher)

        del self.publisher_sensors
        del self.pub_sensor_settings
        return TransitionCallbackReturn.SUCCESS
