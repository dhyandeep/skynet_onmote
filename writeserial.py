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
   timeout=0)
print("connected to: " + ser.portstr)
#ser.write("help\n");

f=open("read.txt","w");
Matrix = [[0 for x in range(length)] for x in range(height)]
for i in range(height):
	for j in range(length):
		Matrix[i][j]=(i+j)%20
		ser.write(str(Matrix[i][j]))
		time.sleep(.005)
		ser.write(" ")
		print(Matrix[i][j]),
	print("\n"),
i=0
j=0
for i in range(0,height):
	for j in range(0,length):
		Matrix[i][j]=(i+j)%14
		ser.write(str(Matrix[i][j]))
		time.sleep(.005)
		ser.write(" ")
		print(Matrix[i][j]),
	print("\n"),
f.close();
