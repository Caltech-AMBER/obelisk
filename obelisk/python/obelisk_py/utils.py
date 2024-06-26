import inspect
from types import ModuleType
from typing import Type, TypeVar, Union, get_args

from rclpy.node import Node


def get_classes_in_module(module: ModuleType) -> list[Type]:
    """Get all classes in a module.

    Parameters:
        module: The module to search.

    Returns:
        A list of classes in the module.
    """
    classes = [getattr(module, member) for member in dir(module) if inspect.isclass(getattr(module, member))]
    return classes


def check_and_get_obelisk_msg_type(
    node: Node, msg_type_name: str, msg_module_or_type: Union[ModuleType, TypeVar]
) -> Type:
    """Check if a message type is in a module and add it as an attribute to a node in place.

    Parameters:
        node: The node to add the attribute to.
        msg_type_name: The name of the message type to add.
        msg_module_or_type: The module over which we verify the message type. If a TypeVar is passed, we verify that the
            message type is in the bound of the TypeVar.

    Returns:
        The message type.

    Raises:
        AssertionError: If the message type is not in the module.
    """
    msg_type = None

    if isinstance(msg_module_or_type, TypeVar):
        msg_module_type_names = [a.__name__ for a in get_args(msg_module_or_type.__bound__)]
        assert msg_type_name in msg_module_type_names, f"{msg_type_name} must be one of {msg_module_type_names}"

        for a in get_args(msg_module_or_type.__bound__):
            if msg_type_name == a.__name__:
                msg_type = a
                break
    else:
        msg_module_type_names = [t.__name__ for t in get_classes_in_module(msg_module_or_type)]
        assert msg_type_name in msg_module_type_names, f"{msg_type_name} must be one of {msg_module_type_names}"

        for msg_module_type_name in msg_module_type_names:
            if msg_type_name == msg_module_type_name:
                msg_type = getattr(msg_module_or_type, msg_type_name)
                break

    assert msg_type is not None, f"Could not find {msg_type_name} in {msg_module_or_type}!"
    return msg_type
