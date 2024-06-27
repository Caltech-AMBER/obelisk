from typing import Dict, Generator, List

import pytest
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter


def init_rclpy() -> None:
    """Initialize rclpy if not already initialized."""
    if not rclpy.ok():
        rclpy.init()


def shutdown_rclpy() -> None:
    """Shutdown rclpy if it's running."""
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture(scope="session", autouse=True)
def ros_context() -> Generator[None, None, None]:
    """Fixture to handle ROS context initialization and shutdown."""
    init_rclpy()
    yield
    shutdown_rclpy()


@pytest.fixture
def set_node_parameters() -> None:
    """Fixture to provide a function for setting node parameters."""

    def _set_node_parameters(node: Node, parameter_dict: Dict) -> None:
        parameters = []
        for name, value in parameter_dict.items():
            if isinstance(value, list):
                param_type = Parameter.Type.STRING_ARRAY
            elif isinstance(value, int):
                param_type = Parameter.Type.INTEGER
            elif isinstance(value, str):
                param_type = Parameter.Type.STRING
            else:
                raise ValueError(f"Unsupported parameter type for {name}")

            parameters.append(Parameter(name, param_type, value))

        node.set_parameters(parameters)

    return _set_node_parameters


@pytest.fixture
def check_node_attributes() -> None:
    """Fixture to provide a function for checking node attributes."""

    def _check_node_attributes(node: Node, expected_attributes: List, should_exist: bool = True) -> None:
        for attr in expected_attributes:
            if should_exist:
                assert hasattr(node, attr), f"Node should have attribute {attr}"
            else:
                assert not hasattr(node, attr), f"Node should not have attribute {attr}"

    return _check_node_attributes
