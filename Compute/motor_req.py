import speed
from constants import *
from utils import clamp_assert


class Motor_req:
    def __init__(self):
        self._power_limit_kw: float = 0.0
        self._torque_lim_pos_Nm: float = 0.0
        self._torque_lim_neg_Nm: float = 0.0
        self._speed_req: speed.Speed = speed.Speed()
        self._torque_mode: bool = False
        self._torque_Nm: float = 0.0
        self._drive_enabled: bool = False

    # --- Positive Torque Limit Property ---
    @property
    def torque_lim_pos_Nm(self) -> float:
        return self._torque_lim_pos_Nm

    @torque_lim_pos_Nm.setter
    def torque_lim_pos_Nm(self, value: float):
        self._torque_lim_pos_Nm = clamp_assert(0, value, MAX_MOTOR_TORQUE_NM)

    # --- Negative Torque Limit Property ---
    @property
    def torque_lim_neg_Nm(self) -> float:
        return self._torque_lim_neg_Nm

    @torque_lim_neg_Nm.setter
    def torque_lim_neg_Nm(self, value: float):
        self._torque_lim_neg_Nm = clamp_assert(-MAX_MOTOR_TORQUE_NM, value, 0)

    # --- Power Limit Property ---
    @property
    def power_limit_kw(self) -> float:
        return self._power_limit_kw

    @power_limit_kw.setter
    def power_limit_kw(self, value: float):
        self._power_limit_kw = clamp_assert(0, value, MAX_MOTOR_POWER_KW)

    # --- Torque Mode Property ---
    @property
    def torque_mode(self) -> bool:
        return self._torque_mode

    @torque_mode.setter
    def torque_mode(self, value: bool):
        self._torque_mode = bool(value)

    # --- Drive Enabled Property ---
    @property
    def drive_enabled(self) -> bool:
        return self._drive_enabled

    @drive_enabled.setter
    def drive_enabled(self, value: bool):
        self._drive_enabled = bool(value)

    # --- Torque Request Property ---
    @property
    def torque_Nm(self) -> float:
        return self._torque_Nm

    @torque_Nm.setter
    def torque_Nm(self, value: float):
        self._torque_Nm = clamp_assert(-MAX_MOTOR_TORQUE_NM, value, MAX_MOTOR_TORQUE_NM)

    # --- Speed Property ---
    @property
    def speed_req(self) -> speed.Speed:
        return self._speed_req

    @speed_req.setter
    def speed_req(self, value: speed.Speed):
        rpm = clamp_assert(0, value.get_motor_rpm(), MAX_MOTOR_RPM)
        self._speed_req.set_motor_rpm(rpm)
