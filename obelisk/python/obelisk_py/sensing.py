from obelisk_py.node import ObeliskNode


class ObeliskSensor(ObeliskNode):
    """Obelisk sensor node.

    Obelisk sensors interface directly with sensing hardware. This could mean that this node runs from the robot, runs
    from some offboard computer which connects to the sensors, or anything else. ObeliskSensors don't nominally need to
    subscribe to any topics. They simply expect to publish some number of sensor messages.

    [NOTE] In derived classes, you should declare settings for sensor publishers. This base class exists only for
    semantic reasons. The user is responsible for implementing the logic for the sensor.
    """
