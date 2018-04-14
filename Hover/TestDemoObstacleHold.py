#Version 0.1
#Authuor: 	Zach Burke
#Date:		02/14/2017

import RPi.GPIO as GPIO
import VL53L0X
from time import sleep
from CanaryComm import CanaryComm

def inRange(dist):
	if(dist >= 2 and dist <= 200):
		return True
	else:
		return False


GPIO.setmode(GPIO.BOARD)

#--------Test Settings--------------------------
HOVER_THROTTLE = 1500
THROTTLE_RANGE = 100
PITCH = 1500
PITCH_RANGE = 50
ROLL = 1500
ROLL_RANGE = 50
SETPOINT = 45
DEADBAND = 5

PITCH_SETPOINT = 50
ROLL_SETPOINT = 50
#------------------------------------------------

#-------------ToF Sensor Initialization-----------------------------------
# GPIO for Sensor 1 shutdown pin
sensor1_shutdown = 20
# GPIO for Sensor 2 shutdown pin
sensor2_shutdown = 16
# GPIO for Sensor 3 shutdown pin
sensor3_shutdown = 8

GPIO.setwarnings(False)

# Setup GPIO for shutdown pins on each VL53L0X
GPIO.setup(sensor1_shutdown, GPIO.OUT)
GPIO.setup(sensor2_shutdown, GPIO.OUT)
GPIO.setup(sensor3_shutdown, GPIO.OUT)

# Set all shutdown pins low to turn off each VL53L0X
GPIO.output(sensor1_shutdown, GPIO.LOW)
GPIO.output(sensor2_shutdown, GPIO.LOW)
GPIO.output(sensor3_shutdown, GPIO.LOW)

# Keep all low for 500 ms or so to make sure they reset
sleep(0.50)

# Create one object per VL53L0X passing the address to give to
# each.
tofBottom = VL53L0X.VL53L0X(address=0x2B)
tofFront = VL53L0X.VL53L0X(address=0x2D)
tofRight = VL53L0X.VL53L0X(address=0x2E)

# Set shutdown pin high for the first VL53L0X then 
# call to start ranging 
GPIO.output(sensor1_shutdown, GPIO.HIGH)
sleep(0.50)
tofBottom.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)

# Set shutdown pin high for the second VL53L0X then 
# call to start ranging 
GPIO.output(sensor2_shutdown, GPIO.HIGH)
sleep(0.50)
tofFront.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)

# Set shutdown pin high for the third VL53L0X then 
# call to start ranging 
GPIO.output(sensor3_shutdown, GPIO.HIGH)
sleep(0.50)
tofRight.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)
#--------------------------------------------------------------


armDrone = input("Arm drone [0 - No, 1 - yes]: ")	 # enable drone
if(armDrone == 0):
	exit()

canary = CanaryComm(0x08)
print "arming drone"
canary.arm()
sleep(1)

height = 0
front = 0
back = 0
left = 0
right = 0

throttle = HOVER_THROTTLE
pitch = PITCH
roll = ROLL
canary.setThrottle(throttle)

while True:
	try:
		#Set Throttle
		try:
			dist = tofBottom.get_distance()/10
		except:
			print "Bottom Distance reading error"
		if(inRange(dist)):
			height = dist

		errorHeight = SETPOINT-height
		if(height < SETPOINT-DEADBAND or height > SETPOINT+DEADBAND):
			throttle = HOVER_THROTTLE + THROTTLE_RANGE*(errorHeight/SETPOINT)
			if(throttle > HOVER_THROTTLE + THROTTLE_RANGE):
				throttle = HOVER_THROTTLE + THROTTLE_RANGE
			elif(throttle < HOVER_THROTTLE - THROTTLE_RANGE):
				throttle = HOVER_THROTTLE = THROTTLE_RANGE
		else:
			throttle = HOVER_THROTTLE
		print "height: ",height," cm"
		print "throttle: ",throttle,"\n"
		
		try:
			canary.setThrottle(throttle)
		except:
			print "Throttle set error"

		#Set Pitch
		try:
			dist = tofFront.get_distance()/10
		except:
			print "Pitch Distance reading error"
		if(inRange(dist)):
			front = dist

		errorFront = front - PITCH_SETPOINT #to get pitch direction correct
		if(front < PITCH_SETPOINT - DEADBAND or front > PITCH_SETPOINT + DEADBAND):
			pitch = PITCH + PITCH_RANGE*(errorFront/PITCH_SETPOINT)
			if(pitch > PITCH + PITCH_RANGE):
				pitch = PITCH + PITCH_RANGE
			elif(pitch < PITCH - PITCH_RANGE):
				pitch = PITCH - PITCH_RANGE
		else:
			pitch = PITCH

		print "front: ",front," cm"
		print "pitch: ",pitch,"\n"
		
		try:
			canary.setPitch(pitch)
		except:
			print "Pitch set error"
		
		#Set Roll
		try:
			dist = tofRight.get_distance()/10
		except:
			print "Roll Distance reading error"
		if(inRange(dist)):
			right = dist
		
		errorRight = right - ROLL_SETPOINT
		if(right < ROLL_SETPOINT - DEADBAND or right > ROLL_SETPOINT + DEADBAND):
			roll = roll + ROLL_RANGE*(errorRight/ROLL_SETPOINT)
			if(roll > ROLL + ROLL_RANGE):
				roll = ROLL + ROLL_RANGE
			elif(roll < Roll - ROLL_RANGE):
				roll = ROLL - ROLL_RANGE
		else:
			roll = ROLL
			
		print "right: ",right," cm"
		print "roll: ",roll,"\n"
		
		try:
			canary.setRoll(roll)
		except:
			print "Roll set error"
			
			
	except:
		canary.disarm()
		GPIO.cleanup()
		raise
		exit()

canary.disarm()
GPIO.cleanup()