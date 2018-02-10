#Version 0.1
#Date 02/09/2018
#Author: Zach Burke

import RPi.GPIO as GPIO
from time import sleep
from CanaryComm import CanaryComm
from UltrasonicSensor import UltrasonicSensor

GPIO.setmode(GPIO.BOARD)

sensor = UltrasonicSensor(32,31)
height = 0

setpoint = 30
throttle = 1500
maxThrottle = 1700
minThrottle = 1500

canary = CanaryComm(0x08)
sleep(1)
canary.arm()
sleep(1)

while True:
	try:
		dist = sensor.getDistanceCM()
		if(dist > 4 and dist < 400):
			height = dist

		if(height < setpoint):
			if(throttle < maxThrottle):
				throttle+=10
		else:
			print "Reached setpoint, disarming"
			break
		print "Height: ",height," cm"
		print "throttle: ",throttle
		canary.setThrottle(throttle)
	except KeyboardInterrupt:
		canary.disarm()
		GPIO.cleanup()
		exit()
sleep(1)
while True:
	dist = sensor.getDistanceCM()
	if(dist >4 and dist < 400):
		height = dist
	if(height < 5):
		break
	else:
		if(throttle > minThrottle):
			throttle -= 10
		print "Height: ",height," cm"
		print "throttle: ",throttle
		canary.setThrottle(throttle)
canary.disarm()
GPIO.cleanup()
