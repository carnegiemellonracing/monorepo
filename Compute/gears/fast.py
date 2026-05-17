"""
FAST — accel pedal scales the torque envelope proportionally; speed and
power are requested at their maxima so the inverter only ever sees torque
as the limiting factor.
"""

from typing import TYPE_CHECKING

from constants import MAX_MOTOR_POWER_KW, MAX_MOTOR_RPM, MAX_MOTOR_TORQUE_NM
from gears import GearHandler

if TYPE_CHECKING:
    from car import Car


class FastGear(GearHandler):
    def apply(self, car: "Car") -> None:
        frac = max(0.0, min(1.0, car.driver_input.accel_pct / 100.0))
        torque_lim_Nm = MAX_MOTOR_TORQUE_NM * frac

        car.set_all_motor_drive_enabled(True)
        car.set_all_motor_torque_modes(False)  # speed-with-torque-limit
        car.set_all_motor_torque_limits_pos_Nm(torque_lim_Nm)
        car.set_all_motor_torque_limits_neg_Nm(-torque_lim_Nm)
        car.set_all_motor_speeds_rpm(MAX_MOTOR_RPM)
        car.set_all_motor_power_limits_kw(MAX_MOTOR_POWER_KW)
