from rcl_interfaces.msg import ParameterEvent

from obelisk_py.internal_utils import get_classes_in_module

try:
    import obelisk_control_msgs.msg
    import obelisk_estimator_msgs.msg
    import obelisk_sensor_msgs.msg
except ImportError as err:
    raise ImportError("The obelisk_msgs packages are not installed! Build the packages in obelisk_ws.") from err


OBELISK_MSG_TYPES = (
    get_classes_in_module(obelisk_sensor_msgs.msg)
    + get_classes_in_module(obelisk_control_msgs.msg)
    + get_classes_in_module(obelisk_estimator_msgs.msg)
)
ROS_ALLOWED_MSG_TYPES = [
    ParameterEvent,
]
