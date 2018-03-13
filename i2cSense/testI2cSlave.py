import sys
import time
import smbus

class SonicComm:
    
    def __init__(self, address):
        self.bus = smbus.SMBus(1)
        self.address = address

    def write(self,cmd):
        self.bus.write_i2c_block_data(self.address, 0 , map(ord,cmd))

    def readOne(self):
        self.readArray =  self.bus.read_i2c_block_data(self.address, 3)
        print self.readArray[0]

    def read(self):
        self.readArray =  self.bus.read_i2c_block_data(self.address, 3)
        print self.readArray[0:6]


x = SonicComm(0x52)
while 1:
	x.write("A")
	time.sleep(.01)
	x.read()
	time.sleep(.1)
	x.write("1")
	time.sleep(.01)
	x.readOne()
	time.sleep(.1)
