import smbus
from time import sleep

class SensorComm:
	def __init__(self, address):
		self.bus = smbus.SMBus(1)
		self.address = address
		self.distance = [0,0,0,0,0,0]

	def write(self,cmd):
		try:
			self.bus.write_byte_data(self.address, 0, map(ord,cmd))
		except:
			print "Sensor i2c write error"

	def readAll(self):
		self.write('A')
		sleep(.01)
		try:
			self.distance = self.bus.read_i2c_block_data(self.address, 6)
		except:
			print "Sensor i2c read error"

	# Read a Single Sensor cmd = 0-5, sent as ASCII
	def readSingle(self,cmd):
		self.write(cmd+0x30)
		sleep(.01)
		try:
			self.distance = self.bus.read_i2c_block_data(self.address, 0, 6)
		except:
			print "Sensor i2c read error"
