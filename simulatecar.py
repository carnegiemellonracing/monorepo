import requests
import time
import base64
import numpy as np
import cbor2
import json
import sseclient
import pprint
import threading

cfile = open("can_fmt.json", mode='r')
config = json.loads(cfile.read())
cfile.close()

class SignalGenerator:
    def __init__(self, freq=1, rate=0, dtype=np.float16, scale=1.0, noise=0):
        self.freq = freq
        self.samp = rate
        self.dtype = dtype
        self.noise = noise
        self.scale = scale

    def get(self, t=0, rate=None):
        if not rate: rate = self.samp

        if rate == 0: return np.array([])

        sig = np.sin(np.linspace(t*rate, t*rate + rate - 1, rate) *
            2*np.pi*float(self.freq) / float(rate) )
        sig *= self.scale
        if self.noise > 0: sig += np.random.normal(scale=self.noise, size=len(sig))

        return np.array(sig, dtype=self.dtype).tobytes()

    def setRate(self, rate):
        self.samp = rate

    def setFreq(self, freq):
        self.freq = freq

class EnumGenerator:
    def __init__(self, rate=0, init=0, proto=None, byname=None):
        self.samp = rate
        self.val = init
        if proto: 
            self.info = config['signals'][proto]['enum']
            self.map = [x['name'] for x in self.info]
            self.val = self.map.index(byname)
        else:
            self.map = []

    def setRate(self, rate):
        self.samp = rate

    def setVal(self, val):
        if self.map:
            self.val = self.map.index(val)
        else:
            self.val = val

    def get(self, t=0):
        if self.samp == 0: return np.array([])
        return np.array([self.val]*self.samp, dtype=np.uint8).tobytes()

class VectorGenerator:
    def __init__(self, rate=0, init=0, proto=None, bylist=None):
        self.samp = rate
        self.val = init
        if proto: self.info = config['signals'][proto]['vector']
        if bylist:
            self.setVal(bylist)

    def setRate(self, rate):
        self.samp = rate

    def setVal(self, vlist):
        b = 0
        mult = 1
        for x in vlist:
            b += x*mult
            mult *= 2
        self.val = b

    def get(self, t=0):
        if self.samp == 0: return np.array([])
        return np.array([self.val]*self.samp, dtype=np.uint8).tobytes()

class ParticleTransmit:
    def __init__(self):
        self.url = "https://api.particle.io/v1/devices/events"
        self.fields = {'name':'sim-car-to-web', 'private':'true', 'data':None,
            'access_token':'f18cc8fdcc2678cb8f9b94aa307cf22e5f87c8b3'}
        
    def send(self, data):
        b = cbor2.dumps(data)
        enc = base64.b64encode(b)
        packet = self.fields.copy()
        packet['data'] = enc
        requests.post(self.url, packet)

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
            client = sseclient.SSEClient(r)
            for event in client.events():
                if str(event.event) != "web-to-car": continue

                resp = json.loads(event.data)
                datastr = resp['data']
                obj = cbor2.loads(base64.b64decode(str(datastr)))

                self.callback(obj)
                print("\n--- Received Input from Live ---")
                pprint.pprint(obj)
                print("")

class RAM:
    def __init__(self, cfg=dict(), sigs=dict()):
        self.rates = {0: 0, 1: 1, 2: 5, 3: 10, 4: 50, 5: 100}
        self.types = {'i64': np.double, 'f32': np.float32, 'i32': np.int32, 'u32': np.uint32, 'f16': np.float16, 'u16': np.uint16, 'u8': np.uint8}
        self.cfg = self.generateStore(cfg)
        self.sigs = self.generateSignals(sigs)
        self.ptx = ParticleTransmit()
        self.prx = ParticleReceive(self.processRx)

        self.txthread = threading.Thread(target=self.transmitPeriodic)
        self.txthread.daemon = True
        self.txthread.start()

    def generateStore(self, ovr):
        s = dict()
        for i,x in enumerate(config['signals']):
            s[i] = 0
            if ovr.has_key(i):
                s[i] = ovr[i]
        return s

    def generateSignals(self, ovr):
        s = dict()
        for i,x in enumerate(config['signals']):
            if x.has_key('enum'): s[i] = EnumGenerator(rate=self.rates[self.cfg[i]])
            elif x.has_key('vector'): s[i] = VectorGenerator(rate=self.rates[self.cfg[i]])
            else: s[i] = SignalGenerator(rate=self.rates[self.cfg[i]], dtype=self.types[x['out_type']])

            if ovr.has_key(i):
                s[i] = ovr[i]
                s[i].setRate(self.rates[self.cfg[i]])
        return s

    def processRx(self, obj):
        if obj.has_key("msg"):
            # Requesting send an arbitrary CAN message
            m = obj['msg']
            print("\n--- Arbitrary CAN TX Requested ---")
            print("ID: %03xh" % m['id'])
            print("BUS: %d" % m['bus'])
            print("DATA: %r" % m['data'])
            print("")
        if obj.has_key("pull"):
            # Requesting a full settings dump
            a = []
            for key in self.cfg:
                a.append(key)
                a.append(self.cfg[key])

            string = np.array(a, dtype=np.uint8).tobytes()
            self.ptx.send({'params': string})
        if obj.has_key("params"):
            # Updating parameters with the spec'd values
            string = obj['params']
            for i in range(len(string)/2):
                ind = ord(string[i*2])
                val = ord(string[i*2+1])

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

            time.sleep(1 - (time.time() - t))
            inc +=  1
            
ram = RAM()
while True:
    time.sleep(1)

def findSigIndex(name):
    for i,x in enumerate(config['signals']):
        if x['name'] == name:
            return i

def nameToEnum(signame, enumname, amnt=1):
    ind = findSigIndex(signame)
    enumval = None
    for i,x in enumerate(config['signals'][ind]['enum']):
        if x['name'] == enumname:
            enumval = i

    return np.array([enumval]*amnt, dtype='uint8').tobytes()