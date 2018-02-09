import smbus

class CanaryComm:
	def __init__(self, address):
		# Init in a disarmed state
		self.channel = [1000,1500,1500,1500,1000]
		self.bus = smbus.SMBus(1)
		self.address = address

	# Set Individual Channels
	def setThrottle(self,throttle):
		self.channel[0] = throttle
		self.updateState()
	def setRoll(self,roll):
		self.channel[1] = roll
		self.updateState()
	def setPitch(self,pitch):
		self.channel[2] = pitch
		self.updateState()
	def setYaw(self,yaw):
		self.channel[3] = yaw
		self.updateState()
	def setArm(self,arm):
		self.channel[4] = arm
		self.updateState()

	# Set States
	def arm(self):
		self.channel = [1000,1500,1500,1500,2000]
		self.updateState()

	def disarm(self):
		self.channel = [1000,1500,1500,1500,1000]
		self.updateState()

	def hover(self):
		self.channel = [1650,1500,1500,1500,2000]
		self.updateState()

	# Return State
	def getState(self):
		return self.channel

	# Send State
	def updateState(self):
		command = []
		#j = 0
		for ch in self.channel:
			command.append(ch & 0xFF)
			command.append((ch & 0xFF00)>>8)
			#j += 1
		self.bus.write_i2c_block_data(self.address, 0x00, command)
