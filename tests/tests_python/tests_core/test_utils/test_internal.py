from types import ModuleType
from typing import Type, TypeVar, Union

import obelisk_control_msgs.msg as ocm
import obelisk_estimator_msgs.msg as oem
import obelisk_sensor_msgs.msg as osm
import pytest
from obelisk_control_msgs.msg import PositionSetpoint
from obelisk_estimator_msgs.msg import EstimatedState
from obelisk_sensor_msgs.msg import ObkJointEncoders

from obelisk_py.core.obelisk_typing import ObeliskControlMsg, ObeliskEstimatorMsg, ObeliskSensorMsg
from obelisk_py.core.utils.internal import check_and_get_obelisk_msg_type, get_classes_in_module


class MockModule(ModuleType):
    """Mock module for testing."""

    class TestClass1:
        """Test class."""

        pass

    class TestClass2:
        """Test class."""

        pass

    def test_function() -> None:
        """Test function."""
        pass


mock_module = MockModule("mock_module")


def test_get_classes_in_module() -> None:
    """Test the get_classes_in_module function."""
    classes = get_classes_in_module(ocm)
    assert PositionSetpoint in classes


@pytest.mark.parametrize(
    "msg_type_name, msg_module_or_type, expected_type",
    [
        ("PositionSetpoint", ocm, PositionSetpoint),
        ("EstimatedState", oem, EstimatedState),
        ("ObkJointEncoders", osm, ObkJointEncoders),
    ],
)
def test_check_and_get_obelisk_msg_type_with_module(
    msg_type_name: str, msg_module_or_type: Union[ModuleType, TypeVar], expected_type: Type
) -> None:
    """Test the check_and_get_obelisk_msg_type function with a module.

    This test verifies that the function correctly retrieves the message type from a module.

    Parameters:
        msg_type_name: The name of the message type to check.
        msg_module_or_type: The module containing the message type.
        expected_type: The expected type to be returned.
    """
    result = check_and_get_obelisk_msg_type(msg_type_name, msg_module_or_type)
    assert result == expected_type


def test_check_and_get_obelisk_msg_type_with_typevar() -> None:
    """Test the check_and_get_obelisk_msg_type function with a TypeVar.

    This test verifies that the function correctly checks if a message type is within the bound of a TypeVar.
    """
    result = check_and_get_obelisk_msg_type("PositionSetpoint", ObeliskControlMsg)
    assert result == PositionSetpoint

    result = check_and_get_obelisk_msg_type("EstimatedState", ObeliskEstimatorMsg)
    assert result == EstimatedState

    result = check_and_get_obelisk_msg_type("ObkJointEncoders", ObeliskSensorMsg)
    assert result == ObkJointEncoders


def test_check_and_get_obelisk_msg_type_with_invalid_type() -> None:
    """Test the check_and_get_obelisk_msg_type function with an invalid type.

    This test verifies that the function raises an AssertionError when given an invalid message type.
    """
    with pytest.raises(AssertionError):
        check_and_get_obelisk_msg_type("InvalidClass", mock_module)

    TestTypeVar = TypeVar("TestTypeVar", bound=Union[mock_module.TestClass1, mock_module.TestClass2])
    with pytest.raises(AssertionError):
        check_and_get_obelisk_msg_type("InvalidClass", TestTypeVar)
