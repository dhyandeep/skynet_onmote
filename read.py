#!/usr/bin/python
import serial
import time
from serial import SerialException
length=8
height=8
piclen=64
pichei=64
ser = serial.Serial(
   port='/dev/ttyUSB0',\
   baudrate=115200,\
   parity=serial.PARITY_NONE,\
   stopbits=serial.STOPBITS_ONE,\
   bytesize=serial.EIGHTBITS,\
   timeout=None)
print("connected to: " + ser.portstr)

f=open('read.pgm','w');
Matrix = [[0 for x in range(piclen)] for x in range(pichei)] 

data=0
time.sleep(2)
values=[]		
text=""
packetno=0
time.sleep(2)
while(packetno<64):
	while data <> '<':
		try:
			data = ser.read()
			#print data
		except Exception as e:
			print "connection error"

	while data<>'>':
		try:
			data = ser.read()
			print(data),
		except Exception as e:
			print "connection error"

		if data=='>':
			packetno=packetno+1
			break
		if data:
			text=text+data;
values=text.split()
print "length"
length=0	
length=len(values)
print length
p=0
for i in range(0,piclen,8):
	for j in range(0,pichei,8):
		for k in range(8):
			for l in range(8):
				Matrix[i+k][j+l]=values[p]
				p=p+1
print(Matrix)
f.write("P2\n64 64\n255\n")
for i in range(piclen):
	for j in range(pichei):
		f.write(Matrix[i][j])
		f.write(" ")
	f.write('\n')
f.close();
