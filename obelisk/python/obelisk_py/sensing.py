from typing import get_args

from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

from obelisk_py.node import ObeliskNode
from obelisk_py.obelisk_typing import ObeliskSensorMsg


class ObeliskSensor(ObeliskNode):
    """Obelisk sensor node.

    Obelisk sensors interface directly with sensing hardware. This could mean that this node runs from the robot, runs
    from some offboard computer which connects to the sensors, or anything else. ObeliskSensors don't nominally need to
    subscribe to any topics. They simply expect to publish some number of sensor messages.

    [NOTE] In derived classes, you should declare settings for sensor publishers.
    """

    def __init__(self, node_name: str) -> None:
        """Initialize the Obelisk sensor."""
        super().__init__(node_name)
        self._has_sensor_publisher = False

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the sensor."""
        super().on_configure(state)

        # ensure there is at least one sensor publisher
        for _, _, msg_type in self._obk_pub_settings:
            if msg_type in [a.__name__ for a in get_args(ObeliskSensorMsg.__bound__)]:
                self._has_sensor_publisher = True
                break
        assert self._has_sensor_publisher, "At least one sensor publisher is required in an ObeliskSensor!"
        return TransitionCallbackReturn.SUCCESS
