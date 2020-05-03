import requests
import time
import base64
import numpy as np
import cbor2
import json

cfile = open("can_fmt.json", mode='r')
config = json.loads(cfile.read())
cfile.close()

url = "https://api.particle.io/v1/devices/events"
fields = {'name':'sim-in', 'private':'true', 'data':None,
        'access_token':'f18cc8fdcc2678cb8f9b94aa307cf22e5f87c8b3'}

def findSigIndex(name):
    for i,x in enumerate(config['signals']):
        if x['name'] == name:
            return i

def genSine(freq, samp_rate, scale=1, off=0):
    off *= samp_rate
    data = scale * np.sin(np.linspace(off, off + samp_rate - 1, samp_rate)*
            2*np.pi*float(freq)/float(samp_rate))
    return np.array(data, dtype='float16').tobytes()

def sendPacket(dat):
    b = cbor2.dumps(dat)
    enc = base64.b64encode(b)
    print(enc)
    fields['data'] = enc
    requests.post(url, fields)

def nameToEnum(signame, enumname, amnt=1):
    ind = findSigIndex(signame)
    enumval = None
    for i,x in enumerate(config['signals'][ind]['enum']):
        if x['name'] == enumname:
            enumval = i

    return np.array([enumval]*amnt, dtype='uint8').tobytes()

i = 0
while True:
    sendPacket({0: genSine(1.1, 10, off=i),
        findSigIndex('Car Mode'): nameToEnum('Car Mode','Reverse'),
        findSigIndex('Car State'): nameToEnum('Car State','GLV'),
        4: b'\x00\x01\x02\x03'})

    i += 1
    time.sleep(1)

