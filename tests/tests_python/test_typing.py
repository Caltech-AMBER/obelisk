import obelisk_control_msgs.msg
import obelisk_estimator_msgs.msg
import obelisk_sensor_msgs.msg

from obelisk_py.internal_utils import get_classes_in_module
from obelisk_py.obelisk_typing import ObeliskControlMsg, ObeliskEstimatorMsg, ObeliskMsg, ObeliskSensorMsg


def test_obelisk_types() -> None:
    """Test that the Obelisk types are defined correctly."""
    obelisk_control_msg_types = get_classes_in_module(obelisk_control_msgs.msg)
    obelisk_estimator_msg_types = get_classes_in_module(obelisk_estimator_msgs.msg)
    obelisk_sensor_msg_types = get_classes_in_module(obelisk_sensor_msgs.msg)

    for msg_type in obelisk_control_msg_types:
        assert issubclass(msg_type, ObeliskControlMsg)
        assert issubclass(msg_type, ObeliskMsg)
    for msg_type in obelisk_estimator_msg_types:
        assert issubclass(msg_type, ObeliskEstimatorMsg)
        assert issubclass(msg_type, ObeliskMsg)
    for msg_type in obelisk_sensor_msg_types:
        assert issubclass(msg_type, ObeliskSensorMsg)
        assert issubclass(msg_type, ObeliskMsg)
