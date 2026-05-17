"""
Top-level entry point. Opens the CAN channel, builds the Car + Controller,
and runs the control loop until Ctrl-C.

Run with:
    python main.py
"""

import time

from canlib import canlib

from car import Car
from controller import Controller

# ---- Edit these for your hardware ----
CAN_CHANNEL = 0
BITRATE = canlib.Bitrate.BITRATE_500K
LOG_PATH = "run.asc"
CONTROL_HZ = 200
# --------------------------------------


def main():
    ch = canlib.openChannel(CAN_CHANNEL, canlib.Open.ACCEPT_VIRTUAL)
    ch.setBusOutputControl(canlib.Driver.NORMAL)
    ch.setBusParams(BITRATE)
    ch.busOn()

    car = Car(ch=ch, log_path=LOG_PATH)
    controller = Controller(car)

    car.start_rx()
    period = 1.0 / CONTROL_HZ
    try:
        next_tick = time.monotonic()
        while True:
            controller.update()
            car.tx_all_motor_reqs()
            next_tick += period
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                # Loop overran — resync to now to avoid runaway catch-up.
                next_tick = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        car.stop_rx()
        ch.busOff()
        ch.close()


if __name__ == "__main__":
    main()
