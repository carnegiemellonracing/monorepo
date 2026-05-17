# Constants file for controls code

# Global Parameters to set
COMPETITION_MODE: bool = False

# car parameters
GEAR_RATIO: float = 12.097
WHEEL_RADIUS_M: float = 0.15
CAR_MASS_KG: float = 200.0


# motor limits/parameters
MOTOR_POLE_PAIRS: float = 4.0
MAX_MOTOR_RPM: float = 20000.0
MAX_MOTOR_TORQUE_NM: float = 31.6
MAX_MOTOR_POWER_KW: float = 35.0

# Motor torque constant: shaft Nm produced per peak A of inverter AC current.
# TUNE for the specific motor (typically 0.03-0.10 Nm/Apk for EV motors).
MOTOR_KT_NM_PER_APK: float = 0.037
