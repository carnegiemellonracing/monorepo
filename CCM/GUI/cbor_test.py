import serial
from cbor2 import loads
import datetime

counter = 1

with serial.Serial('/dev/serial0', 115200, timeout=1) as ser:
	while (1):
		line = ser.readline()
		try:
			obj = loads(line)
			print(counter)
			counter += 1
			print("Successfully deserialized CBOR");
			print(obj)
		except:
			print("Failed to deser CBOR")
