import RPi.GPIO as GPIO
import serial
import sys
from threading import Thread
from time import sleep, strftime, time


class CO2Comm:
	CO2ThreadFlag = 1
	def __init__(self, address, printFlag, logOn, logDir):
		self.CO2_PIN = address
		self.printFlag = printFlag
		self.logOn = logOn
		self.ser = serial.Serial("/dev/ttyAMA0")
		GPIO.setmode(GPIO.BOARD)
		GPIO.setup(self.CO2_PIN, GPIO.OUT)
		GPIO.output(self.CO2_PIN, GPIO.LOW)
		# Initialize Cozir CO2 sensor into streaming mode
		self.ser.write("K 1\r\n")
		sleep(.05)
# Logging 
		if self.logOn:
			self.fname = logDir
			self.fname = self.fname+strftime("%Y.%m.%d.%H%M%S")+'.Co2.csv'
			self.f = open(self.fname,'a')

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
		# --------  Logging - CSV File: -----------------------------
			# Time, concentration [ppm], <CR>
			if self.logOn:
				data = str(time())+','+str(self.conc)+',\n'
				self.f.write(data)

	def exit(self):
		CO2ThreadFlag = 0

