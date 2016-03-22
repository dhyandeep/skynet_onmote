#!/usr/bin/python
import serial
import time
import numpy
length=8
height=8
piclen=64
picwid=64
ser = serial.Serial(
   port='/dev/ttyUSB1',\
   baudrate=115200,\
   parity=serial.PARITY_NONE,\
   stopbits=serial.STOPBITS_ONE,\
   bytesize=serial.EIGHTBITS,\
   timeout=None)
print("connected to: " + ser.portstr)
#ser.write("help\n");


Matrix = [[0 for x in range(64)] for x in range(64)]
packet = [[0 for x in range(8)] for x in range(8)]
elements=0
vert=0
l = []
m=0
f=open('input.txt', 'r')
f.readline()
f.readline()
j=0
i=0
a = [0 for x in range(64*64)]

for line in f:
   if line.strip():           # line contains eol character(s)
       n = int(line)
       a[i]=n
       i=i+1 
m=0
j=0
i=0
print a,
f.close()
for i in range(picwid):
	for j in range(piclen):
		Matrix[i][j]=a[j+i*picwid]
i=0
j=0
print Matrix,
vert=0
horiz=0
data=0
while vert < (picwid) :
	while horiz< (piclen) :
		time.sleep(2)
		for i in range(height):
			for j in range(length):
				ser.write(str(Matrix[vert+i][horiz+j]))
				ser.write(" ")
				print(Matrix[vert+i][horiz+j]),
				time.sleep(.01)
		
		ser.flush()
		horiz=horiz+8
		print("\n")
	vert=vert+8

f.close();
