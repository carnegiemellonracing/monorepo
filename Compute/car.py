import threading
import time
from datetime import datetime
from enum import Enum
from typing import IO, Optional

from canlib import Frame, canlib

import dti_protocol as dti
from constants import MAX_MOTOR_RPM, MOTOR_KT_NM_PER_APK
from driver_input import Driver_input
from motor_req import Motor_req
from motor_state import FaultCodes, Motor_state
from utils import clamp_assert


class Corner(Enum):
    FL = "FL"
    FR = "FR"
    RL = "RL"
    RR = "RR"


DEFAULT_NODE_IDS: dict[Corner, int] = {
    Corner.FL: 1,
    Corner.FR: 2,
    Corner.RL: 3,
    Corner.RR: 4,
}


class Car:
    def __init__(
        self,
        ch: Optional[canlib.Channel] = None,
        node_ids: Optional[dict[Corner, int]] = None,
        log_path: Optional[str] = None,
    ):
        self.ch = ch
        self.node_ids: dict[Corner, int] = dict(node_ids) if node_ids else dict(DEFAULT_NODE_IDS)
        self._node_to_corner: dict[int, Corner] = {n: c for c, n in self.node_ids.items()}

        self.motor_reqs: dict[Corner, Motor_req] = {c: Motor_req() for c in Corner}
        self.motor_states: dict[Corner, Motor_state] = {c: Motor_state() for c in Corner}
        self.driver_input: Driver_input = Driver_input()

        # Generic CAN RX cache: {can_id: latest payload bytes}. Populated by
        # the RX thread for every received frame, no special-casing. Decoders
        # (Driver_input, motor-state refresh) pull what they need from here.
        self.last_rx: dict[int, bytes] = {}

        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._rx_thread: Optional[threading.Thread] = None

        self.log_path = log_path
        self._log_file: Optional[IO] = None
        self._log_start_time: float = 0.0
        self._log_lock = threading.Lock()

    def get_motor_req(self, corner: Corner) -> Motor_req:
        return self.motor_reqs[corner]

    # ---------------------------------------------------------------------
    # Bulk setters
    # ---------------------------------------------------------------------
    def set_all_motor_torque_modes(self, torque_mode: bool):
        for c in Corner:
            self.motor_reqs[c].torque_mode = torque_mode

    def set_all_motor_torques_Nm(self, torque_Nm: float):
        for c in Corner:
            self.motor_reqs[c].torque_Nm = torque_Nm

    def set_all_motor_torque_limits_pos_Nm(self, torque_lim_pos_Nm: float):
        for c in Corner:
            self.motor_reqs[c].torque_lim_pos_Nm = torque_lim_pos_Nm

    def set_all_motor_torque_limits_neg_Nm(self, torque_lim_neg_Nm: float):
        for c in Corner:
            self.motor_reqs[c].torque_lim_neg_Nm = torque_lim_neg_Nm

    def set_all_motor_speeds_rpm(self, speed_rpm: float):
        rpm = clamp_assert(0, speed_rpm, MAX_MOTOR_RPM)
        for c in Corner:
            self.motor_reqs[c].speed_req.set_motor_rpm(rpm)

    def set_all_motor_power_limits_kw(self, power_kw: float):
        for c in Corner:
            self.motor_reqs[c].power_limit_kw = power_kw

    def set_all_motor_drive_enabled(self, enabled: bool):
        for c in Corner:
            self.motor_reqs[c].drive_enabled = enabled

    # ---------------------------------------------------------------------
    # RX lifecycle
    # ---------------------------------------------------------------------
    def start_rx(self):
        if self.ch is None or self._rx_thread is not None:
            return
        if self.log_path is not None:
            self._log_file = open(self.log_path, "w", buffering=1)
            self._log_start_time = time.time()
            self._write_asc_header()
        self._stop_event.clear()
        self._rx_thread = threading.Thread(target=self._rx_loop, name="Car-RX", daemon=True)
        self._rx_thread.start()

    def stop_rx(self):
        self._stop_event.set()
        if self._rx_thread is not None:
            self._rx_thread.join(timeout=2.0)
            self._rx_thread = None
        if self._log_file is not None:
            self._log_file.close()
            self._log_file = None

    # ---------------------------------------------------------------------
    # TX
    # ---------------------------------------------------------------------
    def tx_all_motor_reqs(self):
        """Push every Motor_req out to its inverter. Call periodically — faster
        than half the inverter timeout (typically 100 Hz is plenty)."""
        if self.ch is None:
            return
        for corner in Corner:
            self._tx_motor_req(corner)

    def _tx_motor_req(self, corner: Corner):
        req = self.motor_reqs[corner]
        node_id = self.node_ids[corner]

        # Drive enable — keep-alive; inverter blocks output if False (manual §4.3)
        self._send(node_id, dti.CmdPacketId.DRIVE_ENABLE,
                   dti.encode_drive_enable(req.drive_enabled))

        # Per-direction current limits, derived from torque limits.
        pos_lim_A = req.torque_lim_pos_Nm / MOTOR_KT_NM_PER_APK
        neg_lim_A = req.torque_lim_neg_Nm / MOTOR_KT_NM_PER_APK
        self._send(node_id, dti.CmdPacketId.SET_MAX_AC_CURRENT,
                   dti.encode_set_max_ac_current(pos_lim_A))
        self._send(node_id, dti.CmdPacketId.SET_MAX_AC_BRAKE_CURRENT,
                   dti.encode_set_max_ac_brake_current(neg_lim_A))

        # Setpoint — selects the inverter's control mode.
        if req.torque_mode:
            target_A = req.torque_Nm / MOTOR_KT_NM_PER_APK
            self._send(node_id, dti.CmdPacketId.SET_AC_CURRENT,
                       dti.encode_set_ac_current(target_A))
        else:
            target_erpm = int(round(req.speed_req.get_motor_erpm()))
            self._send(node_id, dti.CmdPacketId.SET_ERPM,
                       dti.encode_set_erpm(target_erpm))

    def _send(self, node_id: int, packet_id: dti.CmdPacketId, data: bytes):
        can_id = dti.pack_can_id(int(packet_id), node_id)
        frame = Frame(id_=can_id, data=data, flags=canlib.MessageFlag.STD)
        self.ch.write(frame)
        self._log_frame_asc(frame, direction="Tx")

    # ---------------------------------------------------------------------
    # RX internals
    # ---------------------------------------------------------------------
    def _rx_loop(self):
        while not self._stop_event.is_set():
            try:
                frame = self.ch.read(timeout=100)
            except canlib.CanNoMsg:
                continue
            except canlib.CanError:
                continue
            self._log_frame_asc(frame, direction="Rx")
            self._handle_frame(frame)

    # ---------------------------------------------------------------------
    # Logging — Vector ASC format
    # ---------------------------------------------------------------------
    def _write_asc_header(self):
        dt = datetime.fromtimestamp(self._log_start_time)
        date_str = (
            dt.strftime("%a %b %d %H:%M:%S.")
            + f"{dt.microsecond // 1000:03d}"
            + dt.strftime(" %Y")
        )
        self._log_file.write(f"date {date_str}\n")
        self._log_file.write("base hex  timestamps absolute\n")
        self._log_file.write("internal events logged\n")

    def _log_frame_asc(self, frame: Frame, direction: str):
        if self._log_file is None:
            return
        rel_t = time.time() - self._log_start_time
        is_ext = bool(frame.flags & canlib.MessageFlag.EXT)
        id_str = f"{frame.id:X}{'x' if is_ext else ''}"
        if frame.flags & canlib.MessageFlag.ERROR_FRAME:
            line = f"{rel_t:11.6f} 1  ErrorFrame\n"
        else:
            kind = "r" if frame.flags & canlib.MessageFlag.RTR else "d"
            data_hex = " ".join(f"{b:02X}" for b in bytes(frame.data))
            line = (
                f"{rel_t:11.6f} 1  {id_str:<15} {direction}   {kind} {frame.dlc} {data_hex}\n"
            )
        with self._log_lock:
            self._log_file.write(line)

    def _handle_frame(self, frame: Frame):
        """Capture every received frame's payload into last_rx, keyed by
        CAN ID. No CAN ID is special at this layer — decoding is done later
        by refresh_rx_state(), driven by the control loop."""
        if frame.flags & (canlib.MessageFlag.ERROR_FRAME | canlib.MessageFlag.RTR):
            return
        with self._lock:
            self.last_rx[frame.id] = bytes(frame.data)

    # ---------------------------------------------------------------------
    # Typed-state refresh — pulls latest values out of last_rx into the
    # typed Driver_input / Motor_state objects. Call once per control cycle
    # (the Controller does this).
    # ---------------------------------------------------------------------
    def refresh_rx_state(self):
        with self._lock:
            rx = dict(self.last_rx)  # snapshot so we can release the lock
        self.driver_input.decode(rx)
        for corner, node_id in self.node_ids.items():
            self._refresh_motor_state(self.motor_states[corner], node_id, rx)

    @staticmethod
    def _refresh_motor_state(state: Motor_state, node_id: int, rx: dict[int, bytes]):
        gd1_id = dti.pack_can_id(int(dti.TxPacketId.GENERAL_DATA_1), node_id)
        gd2_id = dti.pack_can_id(int(dti.TxPacketId.GENERAL_DATA_2), node_id)
        gd3_id = dti.pack_can_id(int(dti.TxPacketId.GENERAL_DATA_3), node_id)

        if (data := rx.get(gd1_id)) is not None:
            d = dti.decode_general_data_1(data)
            state.motor_speed.set_motor_erpm(d["erpm"])
            state.duty_cycle_pct = d["duty_cycle_pct"]
            state.DC_voltage_V = d["input_voltage_V"]
        if (data := rx.get(gd2_id)) is not None:
            d = dti.decode_general_data_2(data)
            state.ac_current_A = d["ac_current_A"]
            state.dc_current_A = d["dc_current_A"]
        if (data := rx.get(gd3_id)) is not None:
            d = dti.decode_general_data_3(data)
            state.controller_temp_C = d["controller_temp_C"]
            state.motor_temp_C = d["motor_temp_C"]
            try:
                state.fault_codes = FaultCodes(d["fault_code"])
            except ValueError:
                pass
