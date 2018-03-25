# File: pidtest.py
# Version: 0.2
# Author: 2018 - Patrick Cote
# Description: Script to test basic hover controller
#		Author:			Date:			Note:
#		Patrick Cote	2/08/2018		Add moving average and error handling
#		Patrick Cote	2/09/2018		Add PID Controller and logging
#		Patrick Cote	3/03/2018		Modified for basic k testing
#		PCote			3/09/2018		PID Implement Fix

import time
from time import sleep
import RPi.GPIO as GPIO
from CanaryComm import CanaryComm
from UltrasonicSensor import UltrasonicSensor
import serial

# Parameters
droneOn = input("Arm drone [0 - No, 1 - yes]: ")	 # enable drone
logOn = 1	   # enable data logging

# Test Settings
setpoint = 50 # [cm]
THOVER = 1610 # Initial Throttle
TMAX = 1700	 # Max throttle value
TMIN = 1575 # Min throttle value
testDur = 20 # Length of test [s]
SMA_LENGTH = 3  # moving average taps

# Controller Gains
Kp = input("Kp Gain : ")		# Proportional gain
Ki = input("Ki Gain : ")		# Integral gain
Kd = input("Kd Gain : ")		# Derivative gain
fs = 10							# Sample Rate [samples/sec]



# Init
GPIO.setmode(GPIO.BOARD)
sensor = UltrasonicSensor(32,31)
ZMIN = 0		# Minimum valid measured height [cm]
ZMAX = 250	  # Maximum valid measured height [cm]
height = 0
throttle = 0
dt = 0.33
distArray = [0]*SMA_LENGTH
dist = 0
dErr = 0
iErr = 0
# Display Current Test Plan
print "---- Test Plan ----"
print "Set point: ",setpoint
print "Throttle Range of ",TMIN,"-",TMAX," for ",testDur,"s"
print "P: ",Kp," I: ",Ki," Kd: ",Kd
# Confirm Drone Arming
if droneOn:
	droneOn = input("Confirm drone arm [0 - No, 1 - yes]: ")	 # enable drone

if droneOn:
	canary = CanaryComm(0x08)
	sleep(1)
	canary.arm()
	sleep(2)
	# Take off Sequence
	try:
		canary.setThrottle(THOVER*.6)
		sleep(.15)
		canary.setThrottle(THOVER*.7)
		sleep(.15)
		canary.setThrottle(THOVER*.8)
		sleep(.15)
		canary.setThrottle(THOVER*.9)
		sleep(.15)
		canary.setThrottle(THOVER)
	except KeyboardInterrupt:
		canary.disarm()
		exit()

try:
	distIn = sensor.getDistanceCM()
except:
	print "Error reading Sensor"
	if input("Leave program [0 for no]:"):
		exit()
	distIn = 0

# Delay after setting the initial hover throttle to avoid ground effect
# interfering with controller testing
sleep(4)


if logOn:
	fname = 'logs/pidtests/'
	if droneOn == 0:
		fname = fname+'x'
	fname = fname+time.strftime("%Y.%m.%d.%H%M%S")+'.S'+str(setpoint)
	fname = fname+'Tl'+str(TMIN)+'Th'+str(TMAX)+'A'+str(SMA_LENGTH)
	fname = fname+'PID'+str(Kp)+'-'+str(Ki)+'-'+str(Kd)+'.csv'
	f = open(fname,'a')

tstart = time.time()

throttle = THOVER

while True:
	try:
		# Get current distance
		try:
			distIn = sensor.getDistanceCM()
		except:
			pass
		
		# Moving Average Filter
		if(distIn >= ZMIN and distIn <= ZMAX):
			distArray.append(distIn)
			del distArray[0]
			height = sum(distArray)/SMA_LENGTH
		
		# Controller
		error = (setpoint-height)
		iErr = iErr + error*dt
		dErr = (error-eprev)/dt
		eprev = dErr
		Tpid = Kp*error+Ki*iErr+Kd*dErr + throttle
		
		# Limit Throttle Values
		throttle = int(min(max(Tpid,TMIN),TMAX))
		
		# Data Output and Logging - CSV File:
		# Time, height, throttle, raw dist, error, controller out, <CR>
		if logOn:
			data = str(time.time())+','+str(height)+','+str(throttle)+','+str(distIn)+','
			data = data + str(error)+','+str(Tpid)+','
			data = data+'\n'
			f.write(data)
		print "distIn: ",distIn," -- height: ",height," cm"
		print "throttle: ",throttle," -- Output: ",Tpid
		print "error: ",error
		
		sleep(.1)
		
		if droneOn:
			canary.setThrottle(throttle)
		
		if time.time()>(tstart+testDur):
			if droneOn:
				print "\nCanary Landing..."
				canary.setThrottle(1550)
				sleep(1)
				canary.setThrottle(1500)
				sleep(2)
				print "\nCanary Disarm"
				canary.disarm()
			f.close()
			GPIO.cleanup()
			print "\nTest Completed"
			exit()
	except KeyboardInterrupt:
		if droneOn:
			print "\nCanary Disarm"
			canary.disarm()
			f.close()
		GPIO.cleanup()
		if logOn:
			f.close()
		print "\nKeyboard Exit"
		exit()
GPIO.cleanup()
if logOn:
	f.close()
