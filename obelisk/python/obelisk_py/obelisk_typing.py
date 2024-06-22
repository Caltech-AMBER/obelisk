from typing import Union

import obelisk_control_msgs.msg as ocm
import obelisk_estimator_msgs.msg as oem
import obelisk_sensor_msgs.msg as osm
from rcl_interfaces.msg import ParameterEvent

# defining custom Obelisk types
ObeliskControlMsg = ocm.PositionSetpoint  # [NOTE] make Union if more types are added later
ObeliskEstimatorMsg = oem.EstimatedState  # [NOTE] make Union if more types are added later
ObeliskSensorMsg = Union[
    osm.JointEncoder,
    osm.JointEncoders,
    osm.TrueSimState,
]
ObeliskMsg = Union[ObeliskControlMsg, ObeliskEstimatorMsg, ObeliskSensorMsg]
ROSAllowedMsg = ParameterEvent  # [NOTE] make Union if more types are added later
ObeliskAllowedMsg = Union[ObeliskMsg, ROSAllowedMsg]
