from enum import Enum

import speed
from constants import *


class FaultCodes(Enum):
    NO_ERR = 0
    OVERVOLTAGE = 1
    UNDERVOLTAGE = 2
    DRVIE = 3
    ABS_OVERCURRENT = 4
    CNTRL_OVERTEMP = 5
    MOTOR_OVERTEMP = 6
    SENSOR_WIRE_FLT = 7
    SENSOR_GENERAL_FLT = 8
    CAN_CMD_ERROR = 9


class Motor_state:
    def __init__(self):
        self.motor_speed: speed.Speed = speed.Speed()
        self.DC_voltage_V: float = 0
        self.duty_cycle_pct: float = 0
        self.ac_current_A: float = 0
        self.dc_current_A: float = 0
        self.controller_temp_C: float = 0
        self.motor_temp_C: float = 0
        self.fault_codes: FaultCodes = FaultCodes.NO_ERR
