#Version 0.1
#Authuor: 	Zach Burke
#Date:		12/04/2017

import smbus

class DroneCommand():
	def __init__(self):
		self.bus = smbus.SMBus(1)
		self.addr = 0x08
		
	def setPitch(self, direction):
		if(direction > 0):
			direction = [0x72,0x06]
			self.bus.write_i2c_block_data(self.addr, 0x00, direction)
		