#!/usr/bin/python
import serial
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
ser.write("help\n");

f=open("read.txt","w");
Matrix = [[0 for x in range(length)] for x in range(height)] 

data=0
while data <> "<":
	data = ser.read();
text=""
while data<>">":
	data = ser.read();
	if data==">":
		break
	if data:
		print(data),
		f.write(data)
		text=text+data;
	
values=text.split()
i=0
while i < len(values):
	print values[i]
	i=i+1
p=0
for i in range(0,height,8):
	for j in range(0,length,8):
		for k in range(0,8):
			for l in range(0,8):
				Matrix[i+k][j+l]=values[p]
				p=p+1
print(Matrix)
f.close();
