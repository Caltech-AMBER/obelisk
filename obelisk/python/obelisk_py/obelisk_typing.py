from typing import List, Type, TypeVar, Union, get_args, get_origin

import obelisk_control_msgs.msg as ocm
import obelisk_estimator_msgs.msg as oem
import obelisk_sensor_msgs.msg as osm
from rcl_interfaces.msg import ParameterEvent

from obelisk_py.utils import get_classes_in_module


def _create_union_type(classes: List[Type]) -> Type:
    """Create a Union type from a list of classes (internal typing util).

    This is a hack for the Union type, which does not accept a list of classes as an argument. This also means that it's
    not technically compatible with static type checking, since the type of the Union is not known before computing the
    Union. However, I couldn't get type checking to work throughout the repo without this hack.
    """
    if not classes:
        raise ValueError("At least one class is required for the Union type")
    return Union[tuple(classes)] if len(classes) > 1 else classes[0]  # type: ignore


# ############## #
# EXTERNAL UTILS #
# ############## #


def is_in_bound(type: Type, typevar: TypeVar) -> bool:
    """Check if a type is within the bound of a TypeVar."""
    bound = typevar.__bound__
    if bound is None:
        raise ValueError("The TypeVar does not have a bound.")

    # check if bound is a Union
    if get_origin(bound) is Union:
        union_types = get_args(bound)
        return type in union_types
    else:
        return type == bound


# define Obelisk message types
ocm_classes = get_classes_in_module(ocm)
oem_classes = get_classes_in_module(oem)
osm_classes = get_classes_in_module(osm)

ObeliskControlMsg = TypeVar("ObeliskControlMsg", bound=_create_union_type(ocm_classes))
ObeliskEstimatorMsg = TypeVar("ObeliskEstimatorMsg", bound=_create_union_type(oem_classes))
ObeliskSensorMsg = TypeVar("ObeliskSensorMsg", bound=_create_union_type(osm_classes))
ObeliskMsg = TypeVar("ObeliskMsg", bound=_create_union_type(ocm_classes + oem_classes + osm_classes))
ObeliskAllowedMsg = TypeVar(
    "ObeliskAllowedMsg", bound=_create_union_type(ocm_classes + oem_classes + osm_classes + [ParameterEvent])
)  # [NOTE] ParameterEvent is a special case - all nodes have a ParameterEvent publisher in ROS2
