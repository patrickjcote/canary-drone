import serial
from time import sleep
import sys
import time
import smbus

class SonicComm:
	
	def __init__(self, address):
		self.bus = smbus.SMBus(1)
		self.address = address

	def write(self,data):
		self.bus.write_i2c_block_data(self.address, 0 , data)

	def readOne(self):
		self.readArray =  self.bus.read_i2c_block_data(self.address, 3)
		return self.readArray[0]

	def read(self):
		self.readArray =  self.bus.read_i2c_block_data(self.address, 3)
		a = self.readArray
		print 'Left: {1:3d}    Rear: {3:3d}    Right: {4:3d}    Front: {5:3d}'.format(a[0], a[1], a[2],a[3], a[4], a[5])


ser = serial.Serial("/dev/ttyAMA0")

ser.write("K 1\r\n")
sleep(.05)


x = SonicComm(0x52)
while 1:
	data = [ord('A')]
	x.write(data)
	time.sleep(.01)
	a = x.read()
	time.sleep(.06)
	data = [ord('A')]
	x.write(data)
	time.sleep(.01)
	a = x.read()
	time.sleep(.06)
	data = [ord('A')]
	x.write(data)
	time.sleep(.01)
	a = x.read()
	time.sleep(.06)

	line = ser.readline()
	outStr = "CO2 Concentration: "+str(line[3:6])+"."+str(line[6:8])+"%"
	# print line[0:9]
	print outStr

