#Version 0.1
#Authuor: 	Zach Burke
#Date:		02/14/2017

import RPi.GPIO as GPIO
from time import sleep
from CanaryComm import CanaryComm
from UltrasonicSensor import UltrasonicSensor

def inRange(dist):
	if(dist >= 2 and dist <= 200):
		return True
	else:
		return False


GPIO.setmode(GPIO.BOARD)

usBottom = UltrasonicSensor(32,31)
usFront = UltrasonicSensor(40, 37)
usRear = UltrasonicSensor(26, 29)
#usRight = UltrasonicSensor(38, 35)
#usLeft = UltrasonicSensor(36, 33)

HOVER_THROTTLE = 1575
THROTTLE_RANGE = 75
PITCH = 1500
PITCH_RANGE = 25
ROLL = 1500
ROLL_RANGE = 50
SETPOINT = 20


canary = CanaryComm(0x08)
print "arming drone"
canary.arm()
sleep(1)

height = 0
forward = 0
back = 0
west = 0
east = 0

throttle = HOVER_THROTTLE
pitch = PITCH
roll = ROLL
canary.setThrottle(throttle)
sleep(1)

while True:
	try:
		#Set Throttle
		dist = usBottom.getDistanceCM()
		if(inRange(dist)):
			height = dist

		if(height < SETPOINT-10):
			throttle += 10
			if(throttle > HOVER_THROTTLE + THROTTLE_RANGE):
				throttle -= 10
		elif(height > SETPOINT+10):
			throttle -= 10
			if(throttle < HOVER_THROTTLE - THROTTLE_RANGE):
				throttle += 10

		print "height: ",height," cm"
		print "throttle: ",throttle,"\n"
		canary.setThrottle(throttle)

		#Set Pitch
		front = usFront.getDistanceCM()
		if(inRange(front)):
			forward = front
		rear = usRear.getDistanceCM()
		if(inRange(rear)):
			back = rear
		pitchAvg = (forward + back)/2

		if(forward < 40  and back > 40):
			pitch = PITCH - PITCH_RANGE
		elif(back < 40 and forward > 40):
			pitch = PITCH + PITCH_RANGE
		else:
			pitch = PITCH

		print "forward: ",forward," cm"
		print "back: ",back," cm"
		print "pitch: ",pitch,"\n"
		canary.setPitch(pitch)

		#Set Roll
#		right = usFront.getDistanceCM()
#		if(inRange(right)):
#			east = right
#		left = usRear.getDistanceCM()
#		if(inRange(left)):
#			west = left
#		rollAvg = (east + west)/2
#
#		if(right > rollAvg and left < rollAvg):
#			roll = ROLL + ROLL_RANGE
#		elif(left > rollAvg and right < rollAvg):
#			roll = ROLL - ROLL_RANGE
#
#		print "roll: ",roll
#		canary.setRoll(roll)
	except:
		canary.disarm()
		GPIO.cleanup()
		raise
		exit()

canary.disarm()
GPIO.cleanup()
