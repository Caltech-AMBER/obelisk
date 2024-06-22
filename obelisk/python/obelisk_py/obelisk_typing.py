from typing import Union

import obelisk_control_msgs.msg as ocm
import obelisk_estimator_msgs.msg as oem
import obelisk_sensor_msgs.msg as osm
from rcl_interfaces.msg import ParameterEvent

# defining custom Obelisk types
ObeliskControlMsgType = ocm.PositionSetpoint  # [NOTE] make Union if more types are added later
ObeliskEstimatorMsgType = oem.EstimatedState  # [NOTE] make Union if more types are added later
ObeliskSensorMsgType = Union[
    osm.JointEncoder,
    osm.JointEncoders,
    osm.TrueSimState,
]
ObeliskMsgType = Union[ObeliskControlMsgType, ObeliskEstimatorMsgType, ObeliskSensorMsgType]
ROSAllowedMsgType = ParameterEvent  # [NOTE] make Union if more types are added later
ObeliskAllowedMsgType = Union[ObeliskMsgType, ROSAllowedMsgType]
