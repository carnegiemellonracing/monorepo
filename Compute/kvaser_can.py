"""
Kvaser CANlib wrapper with SYM-driven signal decode.

Dependencies:
    pip install canlib cantools

`canlib` is Kvaser's official Python wrapper around their CANlib SDK
(https://www.kvaser.com/canlib-webhelp/). The SDK itself must be installed
separately (Kvaser drivers + CANlib SDK for your OS).

Usage:
    with KvaserCAN("car.sym", channel=0, bitrate=canlib.Bitrate.BITRATE_500K) as bus:
        bus.send_message("Motor_Req", {"torque_Nm": 12.5, "torque_mode": 1})
        time.sleep(0.1)
        rpm = bus.signals["motor_rpm"]
        print(rpm.value, rpm.last_rx_time)
"""

import logging
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional

import cantools
from canlib import Frame, canlib


@dataclass
class SignalValue:
    value: Any = None
    last_rx_time: Optional[float] = None  # wall-clock time.time() of last update
    source_can_id: Optional[int] = None   # CAN ID of the frame that last updated this


class KvaserCAN:
    def __init__(
        self,
        sym_path: str,
        channel: int = 0,
        bitrate: int = canlib.Bitrate.BITRATE_500K,
        *,
        can_fd: bool = False,
        rx_timeout_ms: int = 100,
    ):
        self.db = cantools.database.load_file(sym_path)
        self._channel_no = channel
        self._bitrate = bitrate
        self._can_fd = can_fd
        self._rx_timeout_ms = rx_timeout_ms

        # Pre-populate so callers can safely check `bus.signals[name]`
        # even before the first frame arrives.
        self.signals: Dict[str, SignalValue] = {
            sig.name: SignalValue()
            for msg in self.db.messages
            for sig in msg.signals
        }

        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._rx_thread: Optional[threading.Thread] = None
        self._ch: Optional[canlib.Channel] = None
        self._log = logging.getLogger(__name__)

    # -------------------------
    # Lifecycle
    # -------------------------
    def open(self):
        flags = canlib.Open.ACCEPT_VIRTUAL
        if self._can_fd:
            flags |= canlib.Open.CAN_FD

        self._ch = canlib.openChannel(channel=self._channel_no, flags=flags)
        self._ch.setBusOutputControl(canlib.Driver.NORMAL)
        self._ch.setBusParams(self._bitrate)
        self._ch.busOn()

        self._stop_event.clear()
        self._rx_thread = threading.Thread(
            target=self._rx_loop, name="KvaserCAN-RX", daemon=True
        )
        self._rx_thread.start()

    def close(self):
        self._stop_event.set()
        if self._rx_thread is not None:
            self._rx_thread.join(timeout=2.0)
            self._rx_thread = None
        if self._ch is not None:
            try:
                self._ch.busOff()
            finally:
                self._ch.close()
                self._ch = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    # -------------------------
    # TX
    # -------------------------
    def send_message(self, name: str, signals: Dict[str, Any]):
        """Encode by message name from the SYM database and transmit."""
        msg = self.db.get_message_by_name(name)
        data = msg.encode(signals, strict=True)
        flags = canlib.MessageFlag.EXT if msg.is_extended_frame else canlib.MessageFlag.STD
        self._ch.write(Frame(id_=msg.frame_id, data=data, dlc=msg.length, flags=flags))

    def send_raw(self, can_id: int, data: bytes, *, extended: bool = False):
        """Escape hatch: send arbitrary bytes on a raw CAN ID."""
        flags = canlib.MessageFlag.EXT if extended else canlib.MessageFlag.STD
        self._ch.write(Frame(id_=can_id, data=data, flags=flags))

    # -------------------------
    # RX
    # -------------------------
    def _rx_loop(self):
        while not self._stop_event.is_set():
            try:
                frame = self._ch.read(timeout=self._rx_timeout_ms)
            except canlib.CanNoMsg:
                continue
            except canlib.CanError as e:
                self._log.warning("CAN read error: %s", e)
                continue
            self._handle_frame(frame)

    def _handle_frame(self, frame: Frame):
        # Skip error/remote frames — nothing to decode.
        if frame.flags & (canlib.MessageFlag.ERROR_FRAME | canlib.MessageFlag.RTR):
            return
        try:
            msg = self.db.get_message_by_frame_id(frame.id)
        except KeyError:
            return  # frame ID not in SYM
        try:
            decoded = msg.decode(bytes(frame.data))
        except Exception as e:
            self._log.debug("decode failed for id 0x%x: %s", frame.id, e)
            return

        now = time.time()
        with self._lock:
            for name, value in decoded.items():
                sv = self.signals.get(name) or SignalValue()
                sv.value = value
                sv.last_rx_time = now
                sv.source_can_id = frame.id
                self.signals[name] = sv

    # -------------------------
    # Convenience accessors (thread-safe snapshots)
    # -------------------------
    def get(self, name: str) -> SignalValue:
        with self._lock:
            sv = self.signals[name]
            return SignalValue(sv.value, sv.last_rx_time, sv.source_can_id)

    def snapshot(self) -> Dict[str, SignalValue]:
        with self._lock:
            return {
                k: SignalValue(v.value, v.last_rx_time, v.source_can_id)
                for k, v in self.signals.items()
            }
