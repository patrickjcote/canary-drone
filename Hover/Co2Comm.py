import RPi.GPIO as GPIO
import serial
from time import sleep
from threading import Thread
  
class CO2Comm:
	CO2ThreadFlag = 1
	def __init__(self, address, printFlag):
		self.CO2_PIN = address
		self.printFlag = printFlag
		self.ser = serial.Serial("/dev/ttyAMA0")
		GPIO.setmode(GPIO.BOARD)
		GPIO.setup(self.CO2_PIN, GPIO.OUT)
		GPIO.output(self.CO2_PIN, GPIO.LOW)
		self.ser.write("K 1\r\n")
		sleep(.05)
# Sensor read thread Function
		self.flag = 1
		self.CO2Thread = Thread(target=self._CO2Thread)
		self.CO2Thread.start()
		sleep(.01)


	def _CO2Thread(self):
		while self.flag:
			self.line =self.ser.readline()
			self.outStr = "CO2 Concentration: "+str(self.line[3:6])+"."+str(self.line[6:8])+"%"
			self.conc = int(self.line[3:8])
			
			if self.conc > 1000:
				GPIO.output(self.CO2_PIN, GPIO.HIGH)
			else:
				GPIO.output(self.CO2_PIN, GPIO.LOW)
			if self.printFlag:
				print self.outStr
	
	def exit(self):
		CO2ThreadFlag = 0

print "Initializing CO2 Sensor"
a = CO2Comm(40,1)
