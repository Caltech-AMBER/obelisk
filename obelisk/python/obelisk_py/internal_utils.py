import inspect
from types import ModuleType
from typing import Type


def get_classes_in_module(module: ModuleType) -> list[Type]:
    """Get all classes in a module."""
    classes = [getattr(module, member) for member in dir(module) if inspect.isclass(getattr(module, member))]
    return classes
