"""
Top-level vehicle controller. Reads driver input + motor states from Car
and writes per-corner motor requests back into Car. Call `update()` once
per control cycle (~100 Hz).

Gear logic lives in gears/<gear>.py — add a gear by:
  1. adding it to the Gear enum in driver_input.py,
  2. creating gears/<name>.py with a class implementing GearHandler.apply,
  3. registering it in _gear_handlers below.
"""

from car import Car
from driver_input import Gear
from gears import FastGear, GearHandler, NeutralGear


class Controller:
    def __init__(self, car: Car):
        self.car = car
        self._neutral = NeutralGear()
        self._gear_handlers: dict[Gear, GearHandler] = {
            Gear.NEUTRAL: self._neutral,
            Gear.FAST: FastGear(),
        }

    def update(self):
        # Pull latest CAN values into typed state (driver_input + motor_states).
        self.car.refresh_rx_state()
        handler = self._gear_handlers.get(self.car.driver_input.gear, self._neutral)
        handler.apply(self.car)
