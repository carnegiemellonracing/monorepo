# Prerequisites
Running the GUI assumes you're using the following:
* Raspberry Pi 3/4 (tested on 4 but 3 should theoretically work) running Raspbian OS
* Python 3 (preferably >= 3.9) installed
* A working internet connection (at least during setup - running doesn't require internet)

# Quick Start
## Raspberry Pi Setup
These instructions assume you're using Raspbian OS.

You'll need to enable the UART on the Raspberry Pi (instructions stolen from [here](https://www.raspberrypi.org/documentation/configuration/uart.md)):
1. Start raspi-config: `sudo raspi-config`
2. Select option 3 - Interface Options.
3. Select option P6 - Serial Port.
4. At the prompt `Would you like a login shell to be accessible over serial?` answer `No`
5. At the prompt `Would you like the serial port hardware to be enabled?` answer `Yes`
6. Exit raspi-config and reboot the Pi for changes to take effect.

Verify that both `/dev/serial0` and `/dev/serial1` exist. The GUI uses `/dev/serial0` to communicate with the CCM.

Now for wiring. For reference, see [this link](https://www.raspberrypi.org/documentation/usage/gpio/images/GPIO-Pinout-Diagram-2.png) for the Pi 4 pinout.

Connect the TX pin of the CCM's UART to pin 15 (RXD) of the Pi, and the CCM's RX pin to pin 14 (TXD) on the Pi.

## GUI Setup
Below are the instructions for running the GUI
1. Create a venv to avoid polluting the global Python installation: `python3 -m venv .`
2. Enable environment: `. bin/activate`
3. Install Python dependencies: `pip3 install -r requirements.txt`

TODO: Add instructions for adding systemd service to autostart GUI

## Running
Before running always remember to activate the virtual environment.

To launch the GUI, run `python3 ChargerGUIV2.py`.

TODO: Add instructions for using systemd service to autostart GUI

# Updating the GUI
To make it easier to work with the UI, we use `pygubu-designer` (a WYSIWYG editor for Tkinter). If you used the requirements.txt, you should have it installed.

To edit the UI, run `pygubu-designer charger.ui`. You may need to make edits to `ChargerGUIV2.py` depending on what you change.
