# utils file for the code base
from constants import COMPETITION_MODE


def clamp(min_value, value, max_value):
    return max(min_value, min(value, max_value))


def clamp_assert(min_value, value, max_value):
    if COMPETITION_MODE:
        return clamp(min_value, value, max_value)
    assert (
        min_value <= value <= max_value
    ), f"Value {value} is out of bounds [{min_value}, {max_value}]"
    return value
