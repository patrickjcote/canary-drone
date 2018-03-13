# File: hoverController.py
# Version: 0.1 - untested
# Author: 2018 - Patrick Cote
# Description: Hover Controller with i2c sensor read and threading

# Libraries
import RPi.GPIO as GPIO
from time import sleep, strftime, time
from SensorComm import SensorComm
from CanaryComm import CanaryComm
from threading import Thread
import serial

# Init Classes
sensors = SensorComm(0x10)
canary = CanaryComm(0x08)

# --------------- Test Settings------------------------------------------------
armDrone = input("Arm drone [0 - No, 1 - yes]: ")	 # enable drone
logOn = 1		# enable data logging
setpoint = 50	# [cm]
THOVER = 1610	# Initial Throttle
TMAX = 1700		# Max throttle value
TMIN = 1550		# Min throttle value
testDur = 20	# Length of test [s]

# Controller Gains
Kp = input("Kp Gain : ")		# Proportional gain
Ki = input("Ki Gain : ")		# Integral gain
Kd = input("Kd Gain : ")		# Derivative gain

# Display Test Parameters
print "---- Test Plan ----"
print "Set point: ",setpoint
print "Throttle Range of ",TMIN,"-",TMAX," for ",testDur,"s"
print "P: ",Kp," I: ",Ki," Kd: ",Kd

# Confirm Drone Arming
if armDrone:
	armDrone = input("Confirm drone arm [0 - No, 1 - yes]: ")	 # enable drone

# --------------- Init Threading ----------------------------------------------
# Flight Value Thread Function
def _FlighValuesThread():
	global throttle, canary, flightThreadFlag, flightThreadEnable, armDrone
	while flightThreadFlag:
		if flightThreadEnable:
			try:
				canary.setThrottle(throttle)
			except:
				print "Throttle set error"
			sleep(.06)

# Sensor read thread Function
def _SensorThread():
	global height, sensors, sensorThreadFlag
	while sensorThreadFlag:
		height = sensors.readSingle(0)
		sleep(.06)


if armDrone:
	flightThreadFlag = 1
	flightThreadEnable = 0
	flightThread = Thread(target=_FlightValuesThread)
	flightThread.start()
	sleep(.01)

sensorThreadFlag = 1
sensorThread = Thread(target=_SensorThread)
sensorThread.start()
sleep(.01)

# --------------- Takeoff Sequence---------------------------------------------
if armDrone:
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
		sleep(2)
	except KeyboardInterrupt:
		canary.disarm()
		exit()

# --------------- Init Controller ---------------------------------------------
height = sensors.distance[0]
throttle = THOVER
dt = 0.33
dErr = 0
iErr = 0
flightThreadEnable = 1

# --------------- Test Start --------------------------------------------------
tstart = time.time()

while time.time()<(tstart+testDur):
	try:
		height = sensors.distance[0]
		# Controller
		error = (setpoint-height)
		iErr = iErr + error*dt
		dErr = (error-eprev)/dt
		eprev = dErr
		Tpid = Kp*error+Ki*iErr+Kd*dErr + throttle
		
		# Limit Throttle Values
		throttle = int(min(max(Tpid,TMIN),TMAX))
		
		# Logging - CSV File:
		# Time, height, throttle, error, controller out, <CR>
		if logOn:
			data = str(time.time())+','+str(height)+','+str(throttle)+','
			data = data + str(error)+','+str(Tpid)+','
			data = data+'\n'
			f.write(data)
		# Status Output
		print "Height: ",height,"cm","   Error: ",error
		print "Set Throttle: ",throttle,"    Controller Output: ",Tpid
		
		sleep(.1)
		
	except KeyboardInterrupt:
		if armDrone:
			print "\nCanary Disarm"
			canary.disarm()
			flightFlag = 0
		if logOn:
			f.close()
		GPIO.cleanup()
		sensorThreadFlag = 0
		print "\nKeyboard Exit"
		exit()
# --------------- Test Complete   ---------------------------------------------

if armDrone:
	# TODO: Proper landing sequence
	print "\nCanary Landing..."
	canary.setThrottle(1550)
	sleep(1)
	canary.setThrottle(1500)
	sleep(2)
	print "\nCanary Disarm"
	canary.disarm()
	flightThreadFlag = 0
sensorThreadFlag = 0
f.close()
GPIO.cleanup()
if logOn:
	f.close()
print "\nTest Completed"
