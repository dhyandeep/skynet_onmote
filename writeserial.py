#!/usr/bin/python
import serial
import time
length=8
height=8
ser = serial.Serial(
   port='/dev/ttyUSB0',\
   baudrate=115200,\
   parity=serial.PARITY_NONE,\
   stopbits=serial.STOPBITS_ONE,\
   bytesize=serial.EIGHTBITS,\
   timeout=None)
print("connected to: " + ser.portstr)
#ser.write("help\n");

f=open("read.txt","w");
Matrix = [[0 for x in range(length)] for x in range(height)]
elements=0
while elements in range(64):
	time.sleep(.1)
	for i in range(height):
		for j in range(length):
			Matrix[i][j]=(i+j)%20
			ser.write(str(Matrix[i][j]))
			ser.write(" ")
			print(Matrix[i][j]),
			time.sleep(.01)
		print("\n"),
	ser.flush()
	elements=elements+1

f.close();
