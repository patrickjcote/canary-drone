#Version 0.1
#Authuor: 	Zach Burke
#Date:		12/04/2017

import RPi.GPIO as GPIO
from time import sleep
from CanaryComm import CanaryComm
import VL53L0X


canary = CanaryComm(0x08)

class DigitalInput(object):
	def __init__(self, pin):
		self.pin = pin
		GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
		GPIO.add_event_detect(self.pin, GPIO.BOTH)
		GPIO.add_event_callback(self.pin, callback=self.edgeTrigger)
	
	def edgeTrigger(self, pin):
		canary.disarm()
		print('EDGE Detected')
		exit()
			
class IRInput(DigitalInput):
	def __init__(self, pin, direction):
		super(IRInput, self).__init__(pin)
		self.inRange = False
		self.direction = direction
	
	def edgeTrigger(self, pin):
		pass

	def objectInRange(self):
		return not GPIO.input(self.pin)

class TofInput(DigitalInput):
	def __init__(self, pin, direction):
		self.inRange = False
		self.direction = direction
		self.pin = pin
		GPIO.setup(pin, GPIO.OUT)
		GPIO.output(pin, GPIO.LOW)
		# Keep all low for 500 ms or so to make sure they reset
		sleep(0.50)
		self.tof = VL53L0X.VL53L0X(address=pin)
		GPIO.output(pin, GPIO.HIGH)
		sleep(0.50)
		self.tof.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)
		
	def objectInRange(self):
		distance = self.tof.get_distance()
		if distance < 500:
			return True

		
GPIO.setmode(GPIO.BOARD)

#sensors = [IRInput(11, "L"), IRInput(19, "R"), TofInput(36,"F")]
#F, B, R, L, U, D
kill = DigitalInput(40)

sleep(0.5)
canary.arm()
sleep(5)
canary.setThrottle(1700)
sleep(1)
canary.setThrottle(1650)
sleep(1.5)
canary.setThrottle(1600)
sleep(1)
canary.disarm()
