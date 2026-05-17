"""
Gear handlers. Each gear lives in its own module and implements the
GearHandler interface defined here. Add a new gear by creating
gears/<name>.py, subclassing GearHandler, and registering it in Controller.
"""

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from car import Car


class GearHandler(ABC):
    @abstractmethod
    def apply(self, car: "Car") -> None: ...


from gears.fast import FastGear  # noqa: E402
from gears.neutral import NeutralGear  # noqa: E402

__all__ = ["GearHandler", "FastGear", "NeutralGear"]
