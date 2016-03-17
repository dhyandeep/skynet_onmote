#!/usr/bin/python
import serial
ser = serial.Serial(
   port='/dev/ttyUSB0',\
   baudrate=115200,\
   parity=serial.PARITY_NONE,\
   stopbits=serial.STOPBITS_ONE,\
   bytesize=serial.EIGHTBITS,\
   timeout=0)
print("connected to: " + ser.portstr)
ser.write("help\n");

f=open("read.txt","w");
while True:
	data = ser.read();
	if data:
		print(data),
		f.write(data)
		
f.close();
