import smbus
import sys
from time import sleep

class SensorComm:
	def __init__(self, address):
		self.bus = smbus.SMBus(1)
		self.address = address

	def write(self,cmd):
		try:
			data = ord(cmd)
			self.bus.write_byte_data(self.address, 0, data)
		except:
			print "Sensor i2c write error"
			raise

	def readAll(self):
		self.write('A')
		sleep(.01)
		try:
			out = self.bus.read_i2c_block_data(self.address, 3)
			return out[0:5]
		except:
			print "Sensor i2c read error"

	def readSingle(self,cmd):
		self.write(cmd)
		sleep(.01)
		try:
			out = self.bus.read_byte(self.address)
			return out
		except:
			print "Sensor i2c read error"
