"""
NEUTRAL — default safe state. Drive disabled, all limits and setpoints zero.
Also used as the fallback for any unrecognized gear command.
"""

from typing import TYPE_CHECKING

from gears import GearHandler

if TYPE_CHECKING:
    from car import Car


class NeutralGear(GearHandler):
    def apply(self, car: "Car") -> None:
        car.set_all_motor_drive_enabled(False)
        car.set_all_motor_torques_Nm(0)
        car.set_all_motor_torque_limits_pos_Nm(0)
        car.set_all_motor_torque_limits_neg_Nm(0)
        car.set_all_motor_power_limits_kw(0)
        car.set_all_motor_speeds_rpm(0)
