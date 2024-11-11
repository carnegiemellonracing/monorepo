"""
This module simulates a car's CAN (Controller Area Network) signals and transmits them to a web service using Particle's API.
Classes:
    SignalGenerator: Generates sinusoidal signals with optional noise.
    EnumGenerator: Generates enumerated signals based on a predefined configuration.
    VectorGenerator: Generates vector signals based on a predefined configuration.
    ParticleTransmit: Handles transmitting data to a web service using Particle's API.
    ParticleReceive: Handles receiving data from a web service using Particle's API.
    RAM: Manages the configuration and signal generation, and handles periodic transmission of signals.
Functions:
    findSigIndex(name): Finds the index of a signal by its name in the configuration.
    nameToEnum(signame, enumname, amnt=1): Converts a signal name and enum name to a byte array representation.
Usage:
    The module initializes a RAM instance and continuously transmits signals at a specified rate.
"""

import requests
import time
import base64
import numpy as np
import os
from dotenv import load_dotenv
import cbor2
import json
import sseclient
import pprint
import threading

# Load environment variables from .env file
load_dotenv()

cfile = open("can_fmt.json", mode="r")
config = json.loads(cfile.read())
cfile.close()


class SignalGenerator:
    """
    A class used to generate signal data.
    Attributes
    ----------
    freq : float
        Frequency of the signal in Hz (default is 1)
    samp : int
        Sampling rate (default is 0)
    dtype : numpy.dtype
        Data type of the generated signal (default is np.float16)
    scale : float
        Scale factor for the signal amplitude (default is 1.0)
    noise : float
        Standard deviation of Gaussian noise added to the signal (default is 0)
    Methods
    -------
    get(t=0, rate=None)
        Generates a signal for a given time and sampling rate.
    setRate(rate)
        Sets the sampling rate.
    setFreq(freq)
        Sets the frequency of the signal.
    """

    def __init__(self, freq=1, rate=0, dtype=np.float16, scale=1.0, noise=0):
        self.freq = freq
        self.samp = rate
        self.dtype = dtype
        self.noise = noise
        self.scale = scale

    def get(self, t=0, rate=None):
        if not rate:
            rate = self.samp

        if rate == 0:
            return np.array([])

        sig = np.sin(
            np.linspace(t * rate, t * rate + rate - 1, rate)
            * 2
            * np.pi
            * float(self.freq)
            / float(rate)
        )
        sig *= self.scale
        if self.noise > 0:
            sig += np.random.normal(scale=self.noise, size=len(sig))

        return np.array(sig, dtype=self.dtype).tobytes()


class EnumGenerator:
    """
    A class to generate enumerated values at a specified rate.
    Attributes:
    -----------
    samp : int
        The rate at which values are generated.
    val : int
        The current value or the index of the current value in the enumeration map.
    info : list
        The list of enumeration information from the configuration.
    map : list
        The list of enumeration names.
    Methods:
    --------
    __init__(rate=0, init=0, proto=None, byname=None):
        Initializes the EnumGenerator with a rate, initial value, and optional protocol and name.
    setRate(rate):
        Sets the rate at which values are generated.
    setVal(val):
        Sets the current value or the index of the current value in the enumeration map.
    get(t=0):
        Returns the current value repeated 'samp' times as a byte array.
    """

    def __init__(self, rate=0, init=0, proto=None, byname=None):
        self.samp = rate
        self.val = init
        if proto:
            self.info = config["signals"][proto]["enum"]
            self.map = [x["name"] for x in self.info]
            self.val = self.map.index(byname)
        else:
            self.map = []

    def setVal(self, val):
        if self.map:
            self.val = self.map.index(val)
        else:
            self.val = val

    def get(self, t=0):
        if self.samp == 0:
            return np.array([])
        return np.array([self.val] * self.samp, dtype=np.uint8).tobytes()


class VectorGenerator:
    """
    VectorGenerator is a class that generates a vector of bytes based on a given rate and initial value.
    Attributes:
        samp (int): The sampling rate.
        val (int): The initial value or the value set by a list.
        info (dict): Configuration information for the vector, if a prototype is provided.
    Methods:
        __init__(rate=0, init=0, proto=None, bylist=None):
            Initializes the VectorGenerator with a sampling rate, initial value, prototype, or list of values.
        setRate(rate):
            Sets the sampling rate.
        setVal(vlist):
            Sets the value based on a list of integers, converting the list to a single integer.
        get(t=0):
            Returns a byte array of the current value repeated according to the sampling rate.
    """

    def __init__(self, rate=0, init=0, proto=None, bylist=None):
        self.samp = rate
        self.val = init
        if proto:
            self.info = config["signals"][proto]["vector"]

        if bylist:
            self.setVal(bylist)


    def setVal(self, vlist):
        b = 0
        mult = 1
        for x in vlist:
            b += x * mult
            mult *= 2
        self.val = b

    def get(self, t=0):
        if self.samp == 0:
            return np.array([])
        return np.array([self.val] * self.samp, dtype=np.uint8).tobytes()


class ParticleTransmit:
    """
    A class to transmit data to a Particle.io device.
    Attributes
    -----------
    url : str
        The URL endpoint for sending events to Particle.io.
    fields : dict
        A dictionary containing the fields required for the event data.
    Methods
    --------
    __init__():
        Initializes the ParticleTransmit instance with the URL and fields.
    send(data):
        Sends the provided data to the Particle.io device.
    """

    def __init__(self):
        self.url = "https://api.particle.io/v1/devices/events"
        self.fields = {
            "name": "sim-car-to-web",
            "private": "true",
            "data": None,
            "access_token": os.getenv("ACCESS_TOKEN"),
        }

    def send(self, data):
        b = cbor2.dumps(data)
        enc = base64.b64encode(b)
        packet = self.fields.copy()
        packet["data"] = enc
        r = requests.post(self.url, packet)
        if r.status_code != 200:
            print("Error sending data to DAQ Live")
            return

        print("\n--- Sent Data to Live ---")
        pprint.pprint(data)
        print("")


class ParticleReceive:
    def __init__(self, callback):
        self.callback = callback
        self.url = "https://api.particle.io/v1/devices/events?access_token=f18cc8fdcc2678cb8f9b94aa307cf22e5f87c8b3"

        self.rxthread = threading.Thread(target=self.receive)
        self.rxthread.daemon = True
        self.rxthread.start()

    def receive(self):
        with requests.get(self.url, stream=True) as r:
            try:
                client = sseclient.SSEClient(r)
            except requests.exceptions.InvalidURL:
                print("Invalid URL")
                print(r.text)
                return

            for event in client.events():
                if str(event.event) != "web-to-car":
                    continue

                resp = json.loads(event.data)
                datastr = resp["data"]
                obj = cbor2.loads(base64.b64decode(str(datastr)))

                self.callback(obj)
                print("\n--- Received Input from Live ---")
                pprint.pprint(obj)
                print("")


class RAM:
    """
    A class to simulate a RAM module with signal generation and transmission capabilities.
    Attributes:
    -----------
    rates : dict
        A dictionary mapping rate identifiers to their corresponding values.
    types : dict
        A dictionary mapping type identifiers to their corresponding numpy data types.
    cfg : dict
        A dictionary storing the configuration of signals.
    sigs : dict
        A dictionary storing the signal generators.
    ptx : ParticleTransmit
        An instance of ParticleTransmit for transmitting data.
    prx : ParticleReceive
        An instance of ParticleReceive for receiving data.
    txthread : threading.Thread
        A thread for periodic transmission of signals.
    Methods:
    --------
    generateStore(ovr):
        Generates the initial store configuration, optionally overriding with provided values.
    generateSignals(ovr):
        Generates signal generators based on the configuration, optionally overriding with provided values.
    processRx(obj):
        Processes received data and updates configuration or transmits data as requested.
    transmitPeriodic():
        Periodically transmits signal data.
    """
    def __init__(self, cfg=dict(), sigs=dict()):
        self.rates = {0: 0, 1: 1, 2: 5, 3: 10, 4: 50, 5: 100}
        self.types = {
            "i64": np.double,
            "f32": np.float32,
            "i32": np.int32,
            "i16": np.int16,
            "u32": np.uint32,
            "f16": np.float16,
            "u16": np.uint16,
            "u8": np.uint8,
        }
        self.cfg = self.generateStore(cfg)
        self.sigs = self.generateSignals(sigs)
        self.ptx = ParticleTransmit()
        self.prx = ParticleReceive(self.processRx)

        self.txthread = threading.Thread(target=self.transmitPeriodic)
        self.txthread.daemon = True
        self.txthread.start()

    def generateStore(self, ovr):
        s = dict()
        for i, x in enumerate(config["signals"]):
            s[i] = 0
            if i in ovr:
                s[i] = ovr[i]
        return s

    def generateSignals(self, ovr):
        s = dict()
        for i, x in enumerate(config["signals"]):
            if "enum" in x:
                s[i] = EnumGenerator(rate=self.rates[self.cfg[i]])
            elif "vector" in x:
                s[i] = VectorGenerator(rate=self.rates[self.cfg[i]])
            else:
                s[i] = SignalGenerator(
                    rate=self.rates[self.cfg[i]], dtype=self.types[x["out_type"]]
                )

            if i in ovr:
                s[i] = ovr[i]
                s[i].setRate(self.rates[self.cfg[i]])
        return s

    def processRx(self, obj):
        if "msg" in obj:
            # Requesting send an arbitrary CAN message
            m = obj["msg"]
            print("\n--- Arbitrary CAN TX Requested ---")
            print("ID: %03xh" % m["id"])
            print("BUS: %d" % m["bus"])
            print("DATA: %r" % m["data"])
            print("")
        if "pull" in obj:
            # Requesting a full settings dump
            a = []
            for key in self.cfg:
                a.append(key)
                a.append(self.cfg[key])

            string = np.array(a, dtype=np.uint8).tobytes()
            self.ptx.send({"params": string})
        if "params" in obj:
            # Updating parameters with the spec'd values
            string = obj["params"]
            for i in range(int(len(string) / 2)):
                ind = ord(string[i * 2])
                val = ord(string[i * 2 + 1])

                self.cfg[ind] = val
                self.sigs[ind].setRate(self.rates[val])

            # Respond with same values to confirm update
            self.ptx.send(obj)

    def transmitPeriodic(self):
        inc = 0
        while True:
            t = time.time()

            packet = dict()
            for key in self.sigs:
                b = self.sigs[key].get(t=inc)
                if len(b) > 0:
                    packet[key] = b
            self.ptx.send(packet)

            waitTime = 1 - (time.time() - t)
            time.sleep(waitTime if waitTime > 0 else 0)
            inc += 1


def findSigIndex(name):
    for i, x in enumerate(config["signals"]):
        if x["name"] == name:
            return i


def nameToEnum(signame, enumname, amnt=1):
    ind = findSigIndex(signame)
    enumval = None
    for i, x in enumerate(config["signals"][ind]["enum"]):
        if x["name"] == enumname:
            enumval = i

    return np.array([enumval] * amnt, dtype="uint8").tobytes()


if __name__ == "__main__":
    ram = RAM()
    while True:
        time.sleep(1)
