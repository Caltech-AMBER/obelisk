from typing import List, Type, Union

import obelisk_control_msgs.msg
import obelisk_estimator_msgs.msg
import obelisk_sensor_msgs.msg

from obelisk_py.internal_utils import get_classes_in_module


def create_union_type(class_list: List[Type]) -> Union[Type, Type[Type]]:
    """Create a Union type from a list of classes."""
    if not class_list:
        raise ValueError("The class list must not be empty")
    union_type = class_list[0]
    for cls in class_list[1:]:
        union_type = Union[union_type, cls]
    return union_type


# custom types
ObeliskControlMsgType = create_union_type(get_classes_in_module(obelisk_control_msgs.msg))
ObeliskEstimatorMsgType = create_union_type(get_classes_in_module(obelisk_estimator_msgs.msg))
ObeliskSensorMsgType = create_union_type(get_classes_in_module(obelisk_sensor_msgs.msg))
ObeliskMsgType = Union[ObeliskControlMsgType, ObeliskEstimatorMsgType, ObeliskSensorMsgType]
