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
        self.channel = [1000,1518,1489,1500,1750]
        self.updateState()

    def disarm(self):
        self.channel = [1000,1500,1500,1500,1000]
        self.updateState()

    # Return State
    def getState(self):
        return self.channel

    # Send State
    def updateState(self):
        command = []
        for ch in self.channel:
            # Limit Possible Channel Values to 1000-1850
            ch = min(max(ch,1000),1850)
            # Split Channel Value into 2 Bytes
            command.append(ch & 0xFF)
            command.append((ch & 0xFF00)>>8)
        self.bus.write_i2c_block_data(self.address, 0x00, command)
