import smbus
from time import sleep

class SensorComm:
	def __init__(self, address):
		self.bus = smbus.SMBus(1)
		self.address = address
		self.distance = [0,0,0,0,0,0]

	def write(self,cmd):
		self.bus.write_byte_data(self.address, 0, map(ord,cmd))

	def readAll(self):
		self.write('A')
		sleep(.01)
		self.distance = self.bus.read_i2c_block_data(self.address, 6)

	def readSingle(self,cmd):
		self.write(cmd)
		sleep(.01)
		self.distance = self.bus.read_i2c_block_data(self.address, 0, 6)

