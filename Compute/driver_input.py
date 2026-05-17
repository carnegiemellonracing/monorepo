"""
Driver input from CAN: pedal position + currently selected gear.

Decodes itself from Car.last_rx (the generic per-CAN-ID payload cache) so
the Car layer doesn't have to know this ID is special.

CAN format (standard 11-bit ID):
    ID:    0x500
    DLC:   2
    Byte 0: accel_pct  (uint8, 0-100)
    Byte 1: gear       (uint8, Gear enum value)
"""

from enum import Enum


class Gear(Enum):
    NEUTRAL = 0
    FAST = 1
    # Add more gears here; each one gets its own file under gears/.


class Driver_input:
    CAN_ID = 0x500

    def __init__(self):
        self.accel_pct: float = 0.0
        self.gear: Gear = Gear.NEUTRAL

    def decode(self, rx: dict[int, bytes]):
        """Pull the latest driver-input frame out of the CAN RX cache and
        refresh internal state. Missing or short payloads are no-ops, so
        the last-known values persist."""
        data = rx.get(self.CAN_ID)
        if data is None or len(data) < 2:
            return
        self.accel_pct = float(data[0])
        try:
            self.gear = Gear(data[1])
        except ValueError:
            self.gear = Gear.NEUTRAL
