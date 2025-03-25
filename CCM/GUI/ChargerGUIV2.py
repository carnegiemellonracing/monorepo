import tkinter as tk
import serial
import pygubu
import sys
from PIL import Image, ImageTk
from enum import IntEnum
from cbor2 import loads, dumps

# Must be kept in sync with cmr_canHVCMode_t
HVC_MODES = {
    0: 'ERROR',
    (1 << 0): 'IDLE',
    (1 << 1): 'START',
    (1 << 2): 'RUN',
    (1 << 3): 'CHARGE'
}

# Must be kept in sync with cmr_canHVCState_t
HVC_STATES = {
    0x0: 'ERROR',
    0xB: 'CLEAR_ERROR',
    0xC: 'UNKNOWN',
    0x1: 'DISCHARGE',
    0x2: 'STANDBY',
    0x3: 'DRIVE_PRECHARGE',
    0x4: 'DRIVE_PRECHARGE_COMPLETE',
    0x5: 'DRIVE',
    0x6: 'CHARGE_PRECHARGE',
    0x7: 'CHARGE_PRECHARGE_COMPLETE',
    0x8: 'CHARGE_TRICKLE',
    0x9: 'CHARGE_CONSTANT_CURRENT',
    0xA: 'CHARGE_CONSTANT_VOLTAGE'
}


class CCMCommands(IntEnum):
    NONE = 0,
    OFF = 1,
    SLOW = 2,
    FAST = 3,
    RESET = 4


class ChargerUI:
    def __init__(self, serial):
        self.serial = serial
        self.builder = pygubu.Builder()
        self.builder.add_from_file('charger.ui')
        self.mainwindow = self.builder.get_object('mainwindow')
        self.builder.connect_callbacks(self)

        load = Image.open('CMR_LOGO.png')
        # Resize image to fit
        load = load.resize((280, 112), Image.ANTIALIAS)
        self.logo = ImageTk.PhotoImage(load)
        self.builder.get_object('logocanvas').create_image(
            0, 0, image=self.logo, anchor='nw')

    # Updates UI every 10 ms
    def update_data(self):
        line = self.serial.readline()
        try:
            obj = loads(line)
            self.builder.get_object('maxchargecurrent').configure(
                text=str(obj['evse_max_current']))
            self.builder.get_object('hvcstate').configure(
                text=HVC_STATES[obj['hvc']['state']])
            self.builder.get_object('hvcmode').configure(
                text=HVC_MODES[obj['hvc']['mode']])
            if obj['chargers'][0]['state'] == 0:
                self.builder.get_object('charger1status').configure(
                    text='OK', fg='#00ff00')
            else:
                self.builder.get_object('charger1status').configure(
                    text='FAIL', fg='#ff0000')
            if obj['chargers'][1]['state'] == 0:
                self.builder.get_object('charger2status').configure(
                    text='OK', fg='#00ff00')
            else:
                self.builder.get_object('charger2status').configure(
                    text='FAIL', fg='#ff0000')
            self.builder.get_object('charger1voltage').configure(
                text=str(obj['chargers'][0]['voltage'] / 10))
            self.builder.get_object('charger2voltage').configure(
                text=str(obj['chargers'][1]['voltage'] / 10))
            self.builder.get_object('charger1current').configure(
                text=str(obj['chargers'][0]['current'] / 10))
            self.builder.get_object('charger2current').configure(
                text=str(obj['chargers'][1]['current'] / 10))
            self.builder.get_object('hvvoltage').configure(
                text=str(round(obj['hv_voltage'] / 1000, 1)))
            self.builder.get_object('battvoltage').configure(
                text=str(round(obj['batt_voltage'] / 1000, 1)))
            self.builder.get_object('maxcelltemp').configure(
                text=str(round(obj['max_cell_temp'] / 10, 1)))
        except:
            print("Failed to deser CBOR")
        self.mainwindow.after(10, self.update_data)

    def start(self):
        print("Start")
        self.builder.get_object('startbutton').configure(state='disabled')
        self.builder.get_object('stopbutton').configure(state='normal')
        msg = {
            'command': CCMCommands.FAST
        }
        self.serial.write(dumps(msg))

    def stop(self):
        print("Stop")
        self.builder.get_object('startbutton').configure(state='normal')
        self.builder.get_object('stopbutton').configure(state='disabled')
        msg = {
            'command': CCMCommands.OFF
        }
        self.serial.write(dumps(msg))

    def run(self):
        self.mainwindow.after(1, self.update_data)
        self.mainwindow.mainloop()


if __name__ == '__main__':
    try:
        with serial.Serial('/dev/serial0', 115200, timeout=1) as ser:
            app = ChargerUI(ser)
            app.run()
    except:
        print("Unable to open serial device, verify that /dev/serial0 exists")
        sys.exit(1)
