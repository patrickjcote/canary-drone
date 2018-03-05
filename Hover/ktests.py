# File: pidtest.py
# Version: 0.2
# Author: 2018 - Patrick Cote
# Description: Script to test basic hover controller
#		Author:			Date:			Note:
#		Patrick Cote	2/08/2018		Add moving average and error handling
#		Patrick Cote	2/09/2018		Add PID Controller and logging
#		Patrick Cote	3/03/2018		Modified for basic k testing

import time
from time import sleep
import RPi.GPIO as GPIO
from CanaryComm import CanaryComm
from UltrasonicSensor import UltrasonicSensor
import serial

# Parameters
droneOn = input("Arm drone [0 - No, 1 - yes]: ")	 # enable drone
logOn = 1	   # enable data logging
logVerbose = 1  # enable Controller logging

setpoint = 40
THOVER = 1585
TRANGE = 50
testDur = 15

SMA_LENGTH = 3  # moving average taps
Kp = input("Kp Gain [pwm/cm]: ")		# Throttle gain [PWM/cm]
Ki = 0
Kd = 0
fs = 10         # Sample Rate [samples/sec]



# Init
GPIO.setmode(GPIO.BOARD)
sensor = UltrasonicSensor(32,31)
TMIN = THOVER - TRANGE	 # Minimum throttle value
TMAX = THOVER + TRANGE	 # Maximum throttle value
ZMIN = 0		# Minimum valid measured height [cm]
ZMAX = 250	  # Maximum valid measured height [cm]
height = 0
throttle = 0
dt = 0.1
distArray = [0]*SMA_LENGTH
dist = 0
dErr = 0
iErr = 0
print "---- Test Plan ----"
print "Set point: ",setpoint
print "Throttle Range of ",TMIN,"-",TMAX," for ",testDur,"s"
print "P: ",Kp," I: ",Ki," Kd: ",Kd
if droneOn:
	droneOn = input("Confirm drone arm [0 - No, 1 - yes]: ")	 # enable drone

if droneOn:
	canary = CanaryComm(0x08)
	sleep(1)
	canary.arm()
	sleep(2)
	try:
		canary.setThrottle(1500)
		sleep(1)
		canary.setThrottle(1550)
		sleep(1)
		canary.setThrottle(THOVER)
	except KeyboardInterrupt:
		canary.disarm()
		exit()

try:
	Zprev = sensor.getDistanceCM()
except:
	print "Error reading Sensor"
	if input("Leave program [0 for no]:"):
		exit()
	Zprev = 0
# Delay after setting the initial hover throttle to avoid ground effect
# interfering with controller testing
sleep(1)


if logOn:
	fname = 'logs/pidtests/'
	if droneOn == 0:
		fname = fname+'x'
	fname = fname+time.strftime("%Y.%m.%d.%H%M%S")+'.S'+str(setpoint)
	fname = fname+'Tl'+str(TMIN)+'Th'+str(TMAX)+'A'+str(SMA_LENGTH)
	fname = fname+'PID'+str(Kp)+'-'+str(Ki)+'-'+str(Kd)+'.csv'
	f = open(fname,'a')

tstart = time.time()

while True:
	try:
		try:
			distIn = sensor.getDistanceCM()
		except:
			distIn = 0
		# Moving Average Filter
		if(distIn >= ZMIN and distIn <= ZMAX):
			distArray.append(distIn)
			del distArray[0]
			height = sum(distArray)/SMA_LENGTH
		# Controller
		error = (setpoint-height)
		iErr = iErr + error*dt
		dErr = (height-Zprev)/dt
		Zprev = height
		Tpid = (Kp*error+Ki*iErr-Kd*dErr) + THOVER
		# Limit Throttle Values
		throttle = int(min(max(Tpid,TMIN),TMAX))
		# Data Output and Logging - CSV File:
		# Time, height, throttle, raw dist, error, controller out, <CR>
		if logOn:
			data = str(time.time())+','+str(height)+','+str(throttle)+','+str(distIn)+','
			if logVerbose:
				data = data + str(error)+','+str(Tpid)+','
			data = data+'\n'
			f.write(data)
		if logVerbose:
			print "distIn: ",distIn," -- height: ",height," cm"
			print "throttle: ",throttle," -- Output: ",Tpid
			print "error: ",error
		else:
			print "distIn: ",distIn," -- height: ",height," cm"
			print "throttle: ",throttle
		sleep(dt)
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
		print "\nKeyboard Exit"
		exit()
GPIO.cleanup()
if logOn:
	f.close()
