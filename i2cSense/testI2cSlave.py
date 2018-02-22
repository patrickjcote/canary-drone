import sys
import time
import smbus

class SonicComm:
    
    def __init__(self, address):
        self.bus = smbus.SMBus(1)
        self.address = address

    def write(self,cmd):
        self.bus.write_i2c_block_data(self.address, 0 , map(ord,cmd))
    
    def read(self):
        self.readArray =  self.bus.read_i2c_block_data(self.address, 3)
        print self.readArray[0:6]


x = SonicComm(0x52)
x.write("d")
time.sleep(.01)
x.read()
exit()
