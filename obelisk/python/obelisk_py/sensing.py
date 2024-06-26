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
        self.declare_parameter("pub_sensor_config_strs", rclpy.Parameter.Type.STRING_ARRAY)

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the sensor."""
        super().on_configure(state)

        # parsing config strings
        self.pub_sensor_config_strs = (
            self.get_parameter("pub_sensor_config_strs").get_parameter_value().string_array_value
        )
        assert (
            self.pub_sensor_config_strs != [""] and len(self.pub_sensor_config_strs) > 0
        ), "pub_sensor_config_strs must be a non-empty list of strings."

        # create publishers
        self.publisher_sensors = []
        for sensor_config_str in self.pub_sensor_config_strs:
            pub_sensor = self._create_publisher_from_config_str(sensor_config_str)
            self.publisher_sensors.append(pub_sensor)

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Clean up the sensor."""
        super().on_cleanup(state)

        # destroy publishers + config strings
        for sensor_publisher in self.publisher_sensors:
            self.destroy_publisher(sensor_publisher)

        del self.publisher_sensors
        del self.pub_sensor_config_strs
        return TransitionCallbackReturn.SUCCESS
