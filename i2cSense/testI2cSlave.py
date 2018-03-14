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
		print '{0:3d} {1:3d} {2:3d}'.format(a[1], a[2], a[3])


x = SonicComm(0x52)
while 1:
	data = [ord('A')]
	x.write(data)
	time.sleep(.01)
	a = x.read()
	time.sleep(.06)
