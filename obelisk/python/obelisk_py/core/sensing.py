import obelisk_sensor_msgs.msg as osm
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

from obelisk_py.core.node import ObeliskNode
from obelisk_py.core.utils.internal import get_classes_in_module


class ObeliskSensor(ObeliskNode):
    """Obelisk sensor node.

    Obelisk sensors interface directly with sensing hardware. This could mean that this node runs from the robot, runs
    from some offboard computer which connects to the sensors, or anything else. ObeliskSensors don't nominally need to
    subscribe to any topics. They simply expect to publish some number of sensor messages.
    """

    def __init__(self, node_name: str) -> None:
        """Initialize the Obelisk sensor."""
        super().__init__(node_name)
        self._has_sensor_publisher = False

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the sensor."""
        super().on_configure(state)

        # ensure there is at least one sensor publisher
        for pub_dict in self._obk_pub_settings:
            msg_type = pub_dict["msg_type"]
            if msg_type in get_classes_in_module(osm):
                self._has_sensor_publisher = True
                break
        assert self._has_sensor_publisher, (
            "At least one sensor publisher is required in an ObeliskSensor!"
        )
        return TransitionCallbackReturn.SUCCESS
