# File: hoverController.py
# Version: 0.01
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

# Display Current Test Plan
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
	global throttle, canary, flightFlag, armDrone
	while flightThreadOn:
		if flightThreadEnable:
			try:
				canary.setThrottle(throttle)
			except:
				print "Throttle set error"
			sleep(.06)

# Sensor read thread Function
def _SensorThread():
	global height, sensors, sensorFlag
	while sensorFlag:
		height = sensors.readSingle(1)
		sleep(.06)


if armDrone:
	flightThreadOn = 1
	flightThreadEnable = 0
	flightThread = Thread(target=_FlightValuesThread)
	flightThread.start()
	sleep(.01)

sensorFlag = 1
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
	except KeyboardInterrupt:
		canary.disarm()
		exit()

# Delay after setting the initial hover throttle to avoid ground effect
# interfering with controller testing
sleep(4)

# --------------- Init Controller ---------------------------------------------
height = sensors.distance[0]
throttle = THOVER
dt = 0.33
dErr = 0
iErr = 0

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
		# Time, height, throttle, raw dist, error, controller out, <CR>
		if logOn:
			data = str(time.time())+','+str(height)+','+str(throttle)+','+str(distIn)+','
			data = data + str(error)+','+str(Tpid)+','
			data = data+'\n'
			f.write(data)
		# Status Output
		print "distIn: ",distIn," -- height: ",height," cm"
		print "throttle: ",throttle," -- Output: ",Tpid
		print "error: ",error
		
		sleep(.1)
		
	except KeyboardInterrupt:
		if armDrone:
			print "\nCanary Disarm"
			canary.disarm()
			flightFlag = 0
		if logOn:
			f.close()
		GPIO.cleanup()
		sensorFlag = 0
		print "\nKeyboard Exit"
		exit()
# --------------- Test Complete   ---------------------------------------------

if armDrone:
	print "\nCanary Landing..."
	canary.setThrottle(1550)
	sleep(1)
	canary.setThrottle(1500)
	sleep(2)
	print "\nCanary Disarm"
	canary.disarm()
	flightFlag = 0
sensorFlag = 0
f.close()
GPIO.cleanup()
if logOn:
	f.close()
print "\nTest Completed"
