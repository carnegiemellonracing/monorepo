import math
from constants import *


class Speed:
    # internal canonical unit: meters per second
    def __init__(self):
        self.speed_m_s: float = 0.0

    # -------------------------
    # SETTERS (convert → m/s)
    # -------------------------

    def set_speed_m_s(self, speed_m_s: float):
        self.speed_m_s = speed_m_s

    def set_mph(self, mph: float):
        self.speed_m_s = mph / 2.23694

    def set_wheel_rpm(self, wheel_rpm: float):
        # rpm → rad/s → m/s
        wheel_rad_s = (wheel_rpm * 2 * math.pi) / 60
        self.speed_m_s = wheel_rad_s * WHEEL_RADIUS_M

    def set_motor_rpm(self, motor_rpm: float):
        wheel_rpm = motor_rpm / GEAR_RATIO
        self.set_wheel_rpm(wheel_rpm)

    def set_motor_erpm(self, emotor_rpm: float):
        self.set_motor_rpm(emotor_rpm / MOTOR_POLE_PAIRS)

    # -------------------------
    # GETTERS (convert ← m/s)
    # -------------------------

    def get_speed_m_s(self) -> float:
        return self.speed_m_s

    def get_mph(self) -> float:
        return self.speed_m_s * 2.23694

    def get_wheel_rpm(self) -> float:
        wheel_rad_s = self.speed_m_s / WHEEL_RADIUS_M
        return (wheel_rad_s * 60) / (2 * math.pi)

    def get_motor_rpm(self) -> float:
        return self.get_wheel_rpm() * GEAR_RATIO

    def get_motor_erpm(self) -> float:
        return self.get_motor_rpm() * MOTOR_POLE_PAIRS
