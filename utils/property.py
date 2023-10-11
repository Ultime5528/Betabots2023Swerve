from dataclasses import dataclass
from enum import Enum
import inspect
from typing import Optional, Union, Callable

from ntcore import NetworkTableInstance
from ntcore.util import ntproperty as _old_ntproperty


class PropertyMode(Enum):
    Dashboard = "Dashboard"
    ForceDefault = "ForceDefault"
    LocalOnly = "LocalOnly"


@dataclass
class AutopropertyCall:
    key: str
    filename: str
    line_no: int
    col_offset: int


mode = PropertyMode.Dashboard

registry: list[AutopropertyCall] = []

FloatProperty = Union[float, Callable[[], float]]
_DEFAULT_CLASS_NAME = object()


def asCallable(val: FloatProperty) -> Callable[[], float]:
    if callable(val):
        return val
    return lambda: val


def defaultSetter(value):
    pass


def autoproperty(
        default_value,
        key: Optional[str] = None,
        table: Optional[str] = None,
        subtable: Optional[str] = _DEFAULT_CLASS_NAME,
        full_key: Optional[str] = None,
        write: Optional[bool] = None
):
    if mode == PropertyMode.LocalOnly:
        return property(lambda: default_value)

    assert full_key is None or (key is None and table is None and subtable is None)

    curframe = inspect.currentframe()
    calframes = inspect.getouterframes(curframe, 1)
    calframe = calframes[1]

    if full_key is None:
        if table is None:
            table = "Properties"

        if not table.startswith("/"):
            table = "/" + table

        if not table.endswith("/"):
            table += "/"

        if subtable is _DEFAULT_CLASS_NAME:
            subtable = calframe.function

        if subtable is not None:
            table += subtable + "/"

        if key is None:
            code_line = calframe.code_context[0]
            key = code_line.split("=")[0].strip()

        full_key = table + key

    if mode == PropertyMode.ForceDefault:
        write = True
    else:  # PropertyMode.Dashboard, default False (keep saved)
        write = write if write is not None else False

    registry.append(AutopropertyCall(full_key, calframe.filename, calframe.positions.lineno - 1, calframe.positions.col_offset))

    return _old_ntproperty(full_key, default_value, writeDefault=write, persistent=True)
