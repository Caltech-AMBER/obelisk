from typing import TypeVar

import obelisk_control_msgs.msg as ocm
import obelisk_estimator_msgs.msg as oem
import obelisk_sensor_msgs.msg as osm
import pytest
from rcl_interfaces.msg import ParameterEvent

from obelisk_py.core.obelisk_typing import (
    ObeliskAllowedMsg,
    ObeliskControlMsg,
    ObeliskEstimatorMsg,
    ObeliskMsg,
    ObeliskSensorMsg,
    is_in_bound,
)


def test_is_in_bound() -> None:
    """Test the is_in_bound function.

    This test verifies that the is_in_bound function correctly determines if a type is within the bound of a TypeVar.
    """
    assert is_in_bound(ocm.PositionSetpoint, ObeliskControlMsg)
    assert is_in_bound(oem.EstimatedState, ObeliskEstimatorMsg)
    assert is_in_bound(osm.ObkJointEncoders, ObeliskSensorMsg)
    assert is_in_bound(ocm.PositionSetpoint, ObeliskMsg)
    assert is_in_bound(oem.EstimatedState, ObeliskMsg)
    assert is_in_bound(osm.ObkJointEncoders, ObeliskMsg)

    assert not is_in_bound(str, ObeliskControlMsg)
    assert not is_in_bound(int, ObeliskEstimatorMsg)
    assert not is_in_bound(float, ObeliskSensorMsg)
    assert not is_in_bound(bool, ObeliskMsg)


def test_is_in_bound_with_invalid_typevar() -> None:
    """Test the is_in_bound function with an invalid TypeVar.

    This test verifies that the is_in_bound function raises a ValueError when given a TypeVar without a bound.
    """
    InvalidTypeVar = TypeVar("InvalidTypeVar")
    with pytest.raises(ValueError, match="The TypeVar does not have a bound."):
        is_in_bound(str, InvalidTypeVar)


def test_obelisk_allowed_msg_types() -> None:
    """Test the ObeliskAllowedMsg type.

    This test verifies that the ObeliskAllowedMsg type includes the ParameterEvent type in addition to ObeliskMsg types.
    """
    assert ParameterEvent in ObeliskAllowedMsg.__bound__.__args__
    assert all(arg in ObeliskAllowedMsg.__bound__.__args__ for arg in ObeliskMsg.__bound__.__args__)
