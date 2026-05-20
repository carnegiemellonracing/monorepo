"""
DTI HV-500/HV-550/HV-850 inverter CAN protocol (Manual V2.5).

All multi-byte fields are big-endian (Motorola). Every frame is 8 bytes; unused
trailing bytes are padded with 0xFF per the manual.

Standard 11-bit CAN ID:  CANID = (packet_id << 5) | node_id   (packet 6b, node 5b)
Node ID range 1-30; node ID 31 (0x1F) is broadcast.
"""

import struct
from enum import IntEnum


# -----------------------------------------------------------------------------
# Packet IDs
# -----------------------------------------------------------------------------
class TxPacketId(IntEnum):
    """Packets transmitted BY the inverter (we receive these)."""

    GENERAL_DATA_6 = 0x1F  # control mode, target Iq, position, isMotorStill
    GENERAL_DATA_1 = 0x20  # ERPM, duty, input voltage
    GENERAL_DATA_2 = 0x21  # AC current, DC current
    GENERAL_DATA_3 = 0x22  # controller temp, motor temp, fault code
    GENERAL_DATA_4 = 0x23  # Id, Iq (FOC components)
    GENERAL_DATA_5 = 0x24  # throttle/brake, digital I/O, drive enable, limit flags
    AC_CURRENT_LIMITS = 0x25
    DC_CURRENT_LIMITS = 0x26


class CmdPacketId(IntEnum):
    """Packets sent TO the inverter (we transmit these)."""

    SET_AC_CURRENT = 0x01
    SET_BRAKE_CURRENT = 0x02
    SET_ERPM = 0x03
    SET_POSITION = 0x04
    SET_RELATIVE_CURRENT = 0x05
    SET_RELATIVE_BRAKE_CURRENT = 0x06
    SET_DIGITAL_OUTPUT = 0x07
    SET_MAX_AC_CURRENT = 0x08
    SET_MAX_AC_BRAKE_CURRENT = 0x09
    SET_MAX_DC_CURRENT = 0x0A
    SET_MAX_DC_BRAKE_CURRENT = 0x0B
    DRIVE_ENABLE = 0x0C


BROADCAST_NODE_ID = 0x1F


# -----------------------------------------------------------------------------
# CAN ID pack / unpack (standard 11-bit)
# -----------------------------------------------------------------------------
def pack_can_id(packet_id: int, node_id: int) -> int:
    return (packet_id << 5) | (node_id & 0x1F)


def unpack_can_id(can_id: int) -> tuple[int, int]:
    return (can_id >> 5), (can_id & 0x1F)


# -----------------------------------------------------------------------------
# Command encoders (Section 4.2). Each returns the natural DLC payload — the
# inverter accepts short DLCs per the manual ("Fill with FFs or use N-byte DLC").
# -----------------------------------------------------------------------------
def encode_set_ac_current(amps_pk: float) -> bytes:
    """0x01 - Set AC current. Signed s16, scale 10. ±850 Apk operational."""
    return struct.pack(">h", int(round(amps_pk * 10)))


def encode_set_brake_current(amps_pk: float) -> bytes:
    """0x02 - Set brake current. Positive only, scale 10."""
    return struct.pack(">h", int(round(max(0.0, amps_pk) * 10)))


def encode_set_erpm(erpm: int) -> bytes:
    """0x03 - Set speed (electrical RPM). Signed s32, scale 1."""
    return struct.pack(">i", int(erpm))


def encode_set_max_ac_current(amps_pk: float) -> bytes:
    """0x08 - Max AC drive current. Positive, scale 10."""
    return struct.pack(">h", int(round(max(0.0, amps_pk) * 10)))


def encode_set_max_ac_brake_current(amps_pk: float) -> bytes:
    """0x09 - Max AC brake current. Negative only, scale 10."""
    return struct.pack(">h", int(round(min(0.0, amps_pk) * 10)))


def encode_set_max_dc_current(amps: float) -> bytes:
    """0x0A - Max DC current. Positive, scale 10."""
    return struct.pack(">h", int(round(max(0.0, amps) * 10)))


def encode_set_max_dc_brake_current(amps: float) -> bytes:
    """0x0B - Max DC brake current. Negative only, scale 10."""
    return struct.pack(">h", int(round(min(0.0, amps) * 10)))


def encode_drive_enable(enable: bool) -> bytes:
    """0x0C - Drive enable. 1 byte: 1=allow, 0=disallow."""
    return bytes([1 if enable else 0])


# -----------------------------------------------------------------------------
# Telemetry decoders (Section 3.2)
# -----------------------------------------------------------------------------
def decode_general_data_1(data: bytes) -> dict:
    """0x20 - ERPM (s32 scale 1), duty (s16 scale 10 %), input voltage (s16 scale 1 V)."""
    erpm = struct.unpack(">i", data[0:4])[0]
    duty_pct = struct.unpack(">h", data[4:6])[0] / 10.0
    v_in = struct.unpack(">h", data[6:8])[0]
    return {"erpm": erpm, "duty_cycle_pct": duty_pct, "input_voltage_V": float(v_in)}


def decode_general_data_2(data: bytes) -> dict:
    """0x21 - AC current and DC current (both s16, scale 10, A)."""
    ac = struct.unpack(">h", data[0:2])[0] / 10.0
    dc = struct.unpack(">h", data[2:4])[0] / 10.0
    return {"ac_current_A": ac, "dc_current_A": dc}


def decode_general_data_3(data: bytes) -> dict:
    """0x22 - Controller temp, motor temp (s16 scale 10 °C), fault code (u8)."""
    ctlr = struct.unpack(">h", data[0:2])[0] / 10.0
    motor = struct.unpack(">h", data[2:4])[0] / 10.0
    fault = data[4]
    return {"controller_temp_C": ctlr, "motor_temp_C": motor, "fault_code": fault}
