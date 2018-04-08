# File: hoverController.py
# Version: 0.1 - untested
# Author: 2018 - Patrick Cote
# Description: Hover Controller using ToF Sensors

# --------------- Test Settings------------------------------------------------
logOn = 1		# enable data logging
SENSOR_TYPE = 0		# Distance sensor, 0 = ultrasonic, 1 = tof
setpoint = 55	# [cm]
testDur = 10	# Length of test [s]
# Limits
TMAX = 1450		# Max throttle value
TMIN = 1400		# Min throttle value
SMA_LENGTH = 3	# Length of Simple Moving Average
# Time of Flight Mode 0-good,1-better,2-Best,3-Long,4-High
TOF_MODE = 4
MAX_DIST_IN = 200 # 

# -----------------------------------------------------------------------------

# Libraries
import RPi.GPIO as GPIO
from time import sleep, strftime, time
from CanaryComm import CanaryComm
from threading import Thread
if SENSOR_TYPE:
	import VL53L0X
else:
	from SensorComm import SensorComm
#import serial

# Init Classes
canary = CanaryComm(0x08)
sensors = SensorComm(0x52)

#Initialize ToF sensor---------------------------------------------------------
# GPIO for Sensor 1 shutdown pin
sensor1_shutdown = 7
GPIO.setwarnings(False)

# Setup GPIO for shutdown pins on each VL53L0X
GPIO.setmode(GPIO.BOARD)
GPIO.setup(sensor1_shutdown, GPIO.OUT)

# Set all shutdown pins low to turn off each VL53L0X
GPIO.output(sensor1_shutdown, GPIO.LOW)

# Keep all low for 500 ms or so to make sure they reset
sleep(0.50)

# Create one object per VL53L0X passing the address to give to
# each.
if SENSOR_TYPE:
	tof = VL53L0X.VL53L0X(address=0x2B)

# Set shutdown pin high for the first VL53L0X then
# call to start ranging
GPIO.output(sensor1_shutdown, GPIO.HIGH)
sleep(0.50)
if SENSOR_TYPE:
	tof.start_ranging(TOF_MODE)

sleep(0.50)
if SENSOR_TYPE:
	timing = tof.get_timing()
	if (timing < 20000):
		timing = 20000
	dt = timing/1000000.00
else:
	dt = .06
throttleStep = (TMAX-TMIN)/(testDur/dt)
#End ToF sensor initialization--------------------------------------------------

# ------- User Input ----------------------------------------------------------
armDrone = input("Arm drone [0 - No, 1 - yes]: ")	 # enable drone

inAr = sensors.readAll()
print inAr
# Display Test Parameters
print "---- Test Plan ----"
print "Polling Rate: ", 1/dt,"Hz"
print "Throttle Range of ",TMIN,"-",TMAX," for ",testDur,"s"
print "Throttle Step of ",throttleStep

# Confirm Drone Arming
if armDrone:
	armDrone = input("Confirm drone arm [0 - No, 1 - yes]: ")	 # enable drone

# --------------- Takeoff Sequence---------------------------------------------
if armDrone:
	canary = CanaryComm(0x08)
	sleep(1)
	canary.arm()
	sleep(2)
	# Take off Sequence
	try:
		canary.setThrottle(TMIN*.8)
		sleep(.15)
		canary.setThrottle(TMIN*.9)
		sleep(.15)
		canary.setThrottle(TMIN)
		sleep(.15)
	except KeyboardInterrupt:
		canary.disarm()
		exit()
throttle = TMIN
tFloat = TMIN
# --------------- Init Controller ---------------------------------------------
height = 0
distArray = [0]*SMA_LENGTH
# --------------- Test Start --------------------------------------------------
tstart = time()

if logOn:
	fname = 'logs/modelingTests/'
	if armDrone == 0:
		fname = fname+'x'
	fname = fname+strftime("%Y.%m.%d.%H%M%S.")
	fname = fname+'Tl'+str(TMIN)+'Th'+str(TMAX)+'.csv'
	f = open(fname,'a')


while time()<(tstart+testDur):
	try:
		
		# Logging - CSV File:
		# Time, height, throttle, <CR>
		if logOn:
			data = str(time())+','+str(height)+','+str(throttle)+','
			data = data+'\n'
			f.write(data)
		
		# Update Throttle
		# Limit Throttle Values
		tFloat = tFloat+throttleStep
		throttle = int(min(max(tFloat,TMIN),TMAX))
		# Status Output
		print "Height: ",height,"cm, Set Throttle: ",throttle
		try:
			canary.setThrottle(throttle)
		except:
			print "Throttle set error"
		# Update Distance
		try:
			# Read time of flight and convert to cm
			if SENSOR_TYPE:
				distIn = (tof.get_distance())/10
			else:
				inArray = sensors.readSingle('5')
				print inArray
				distIn = inArray
			print distIn
			# Check if in valid range then load into moving average
			if(distIn > -1 and distIn < MAX_DIST_IN):
				distArray.append(distIn)
				del distArray[0]
			height = sum(distArray)/SMA_LENGTH
		except:
			print "Dist error"
			raise
		sleep(dt)
	except KeyboardInterrupt:
		if armDrone:
			print "\nCanary Disarm"
			canary.disarm()
		if logOn:
			f.close()
		if SENSOR_TYPE:
			tof.stop_ranging()
			GPIO.output(sensor1_shutdown, GPIO.LOW)
			GPIO.cleanup()
		print "\nKeyboard Exit"
		exit()
# --------------- Test Complete   ---------------------------------------------

if armDrone:
	# TODO: Proper landing sequence
	print "\nCanary Disarm"
	canary.disarm()
f.close()
if SENSOR_TYPE:
	tof.stop_ranging()
	GPIO.output(sensor1_shutdown, GPIO.LOW)
	GPIO.cleanup()
if logOn:
	f.close()
print "\nTest Completed"
