import obelisk_control_msgs.msg
import obelisk_estimator_msgs.msg
import obelisk_sensor_msgs.msg

from obelisk_py.internal_utils import get_classes_in_module
from obelisk_py.obelisk_typing import (
    ObeliskAllowedMsg,
    ObeliskControlMsg,
    ObeliskEstimatorMsg,
    ObeliskMsg,
    ObeliskSensorMsg,
    is_in_bound,
)


def test_obelisk_types() -> None:
    """Test that the Obelisk types are defined correctly."""
    obelisk_control_msg_types = get_classes_in_module(obelisk_control_msgs.msg)
    obelisk_estimator_msg_types = get_classes_in_module(obelisk_estimator_msgs.msg)
    obelisk_sensor_msg_types = get_classes_in_module(obelisk_sensor_msgs.msg)

    # check the lowest-level types
    for msg_type in obelisk_control_msg_types:
        assert is_in_bound(msg_type, ObeliskControlMsg)
        assert msg_type in ObeliskMsg.__bound__.__args__
        assert msg_type in ObeliskAllowedMsg.__bound__.__args__
    for msg_type in obelisk_estimator_msg_types:
        assert is_in_bound(msg_type, ObeliskEstimatorMsg)
        assert msg_type in ObeliskMsg.__bound__.__args__
        assert msg_type in ObeliskAllowedMsg.__bound__.__args__
    for msg_type in obelisk_sensor_msg_types:
        assert is_in_bound(msg_type, ObeliskSensorMsg)
        assert msg_type in ObeliskMsg.__bound__.__args__
        assert msg_type in ObeliskAllowedMsg.__bound__.__args__
