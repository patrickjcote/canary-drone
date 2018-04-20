# File: hoverController.py
# Version: 0.1 - untested
# Author: 2018 - Patrick Cote
# Description: Hover Controller using ToF Sensors

# --------------- Test Settings------------------------------------------------
logOn = 1		# enable data logging
setpoint = 55	# [cm]
SETPOINT2 = 55	# [cm]   step change in set point halfway through the test
TAKEOFF_PITCH = 1500	# Pitch value to combat ugly takeoff
testDur = 15	# Length of test [s]
# Limits
TMAX = 1575		# Max throttle value
TMIN = 1425		# Min throttle value
TMID = 1500		# Initial Throttle
SMA_LENGTH = 3	# Length of Simple Moving Average
IERR_LIM = 70	# Max +/- integral error for windup reset
# Time of Flight Mode 0-good,1-better,2-Best,3-Long,4-High Speed
TOF_MODE = 3
TOF_MODE_B = 3
MAX_DIST_IN = 300 # 

# -----------------------------------------------------------------------------

# Libraries
import RPi.GPIO as GPIO
import VL53L0X
from time import sleep, strftime, time
from CanaryComm import CanaryComm
from threading import Thread

STATUS_LED = 40
BUTTON_PIN = 38

# Init Status Light and Start Button
GPIO.setmode(GPIO.BOARD)
GPIO.setup(STATUS_LED, GPIO.OUT)
GPIO.setup(BUTTON_PIN, GPIO.IN)
#import serial



#End ToF sensor initialization--------------------------------------------------
#-------------ToF Sensor Initialization-----------------------------------
# GPIO for Sensor 1 shutdown pin
sensor1_shutdown = 37 #DOWN
# GPIO for Sensor 2 shutdown pin
sensor2_shutdown = 35 #FWD

GPIO.setwarnings(False)

# Setup GPIO for shutdown pins on each VL53L0X
GPIO.setup(sensor1_shutdown, GPIO.OUT)
GPIO.setup(sensor2_shutdown, GPIO.OUT)

# Set all shutdown pins low to turn off each VL53L0X
GPIO.output(sensor1_shutdown, GPIO.LOW)
GPIO.output(sensor2_shutdown, GPIO.LOW)

# Keep all low for 500 ms or so to make sure they reset
sleep(0.50)

# Create one object per VL53L0X passing the address to give to
# each.
tofBottom = VL53L0X.VL53L0X(address=0x2B)
tofFront = VL53L0X.VL53L0X(address=0x2D)

# Set shutdown pin high for the first VL53L0X then 
# call to start ranging 
GPIO.output(sensor1_shutdown, GPIO.HIGH)
sleep(0.50)
tofBottom.start_ranging(TOF_MODE_B)

# Set shutdown pin high for the second VL53L0X then 
# call to start ranging 
GPIO.output(sensor2_shutdown, GPIO.HIGH)
sleep(0.50)
tofFront.start_ranging(TOF_MODE)

sleep(0.50)
timing = tofBottom.get_timing()
if (timing < 20000):
    timing = 20000
dt = timing/1000000.00
#--------------------------------------------------------------

# ------- User Input ----------------------------------------------------------
armDrone = input("Arm drone [0 - No, 1 - yes]: ")	 # enable drone

# Controller Gains
Kp = 1
Ki = 0
Kd = 0
#Kp = input("Kp Gain : ")		# Proportional gain
#Ki = input("Ki Gain : ")		# Integral gain
#Kd = input("Kd Gain : ")		# Derivative gain
# Display Test Parameters
print "---- Test Plan ----"
print "Polling Rate: ", 1/dt,"Hz"
print "Set point: ",setpoint
print "Throttle Range of ",TMIN,"-",TMAX," for ",testDur,"s"
print "P: ",Kp," I: ",Ki," Kd: ",Kd

# Confirm Drone Arming
if armDrone:
	armDrone = input("Confirm drone arm [0 - No, 1 - yes]: ")	 # enable drone
	# Init Classes
	canary = CanaryComm(0x08)

print "Press the 'GO Button' on the drone..."
# Wait for button to init test, Flash LED to signal ready
while not GPIO.input(BUTTON_PIN):
	GPIO.output(STATUS_LED, GPIO.HIGH)
	sleep(.5)
	GPIO.output(STATUS_LED, GPIO.LOW)
	sleep(.5)
# Rapid flash Status LED to signal start
for i in range(0,10):
	GPIO.output(STATUS_LED, GPIO.HIGH)
	sleep(.1)
	GPIO.output(STATUS_LED, GPIO.LOW)
	sleep(.1)

# Delay 1 second after Rapid Flash, then start program
sleep(1)
# --------------- Takeoff Sequence---------------------------------------------
height = 0
distArray = [0]*SMA_LENGTH
if armDrone:
	canary = CanaryComm(0x08)
	sleep(1)
	canary.arm()
	sleep(2)
	# Take off Sequence
	try:
		while(height<setpoint*0.75):
			try:
				# Read time of flight and convert to cm
				distIn = (tofBottom.get_distance())/10
				print distIn
				# Check if in valid range then load into moving average
				if(distIn > 0 and distIn < MAX_DIST_IN):
					distArray.append(distIn)
					del distArray[0]
				height = sum(distArray)/SMA_LENGTH
			except:
				print "Dist error"
			throttle = TMAX
			try:
				canary.setThrottle(throttle)
				canary.setPitch(TAKEOFF_PITCH)
			except:
				raise
				print "Throttle set error"
			sleep(.5)
		canary.setThrottle(TMID)
		canary.setPitch(1500)
	except KeyboardInterrupt:
		canary.disarm()
		exit()

# --------------- Init Controller ---------------------------------------------
throttle = TMID
dErr = 0
iErr = 0
heightPrev = height
IMAX = IERR_LIM
IMIN = -IERR_LIM
# --------------- Test Start --------------------------------------------------
tstart = time()

if logOn:
	fname = 'logs/nazaHover/'
	if armDrone == 0:
		fname = fname+'x'
	fname = fname+strftime("%Y.%m.%d.%H%M%S")+'.S'+str(setpoint)
	fname = fname+'Tl'+str(TMIN)+'Th'+str(TMAX)+'A'
	fname = fname+'PID'+str(Kp)+'-'+str(Ki)+'-'+str(Kd)+'.csv'
	f = open(fname,'a')


while time()<(tstart+testDur):
	try:
		if time()>(tstart+testDur/2):
			setpoint = SETPOINT2
		# Controller
		error = (setpoint-height)
		iErr = iErr + error*dt
		# Anti-windup reset
		iErr = min(max(iErr,IMIN),IMAX)
		dErr = (height-heightPrev)/dt
		heightPrev = height
		Tpid = Kp*error+Ki*iErr-Kd*dErr + TMID
		# Limit Throttle Values
		throttle = int(min(max(Tpid,TMIN),TMAX))
		
		# Update Throttle
		if armDrone:
			try:
				canary.setThrottle(throttle)
			except:
				raise
				print "Throttle set error"
		# Update Distance
		try:
			# Read time of flight and convert to cm
			distIn = (tofBottom.get_distance())/10
			print distIn
			# Check if in valid range then load into moving average
			if(distIn > 0 and distIn < MAX_DIST_IN):
				distArray.append(distIn)
				del distArray[0]
			height = sum(distArray)/SMA_LENGTH
		except:
			print "Dist error"

		# --------  Logging - CSV File: -----------------------------
		# Time, height, throttle, error, controller out, <CR>
		if logOn:
			data = str(time())+','+str(height)+','+str(throttle)+','
			data = data + str(error)+','+str(Tpid)+','+str(setpoint)+','
			data = data+'\n'
			f.write(data)
		# Status Output
		print "Height: ",height,"cm","   Error: ",error
		print "Set Throttle: ",throttle,"    Controller Output: ",Tpid
		print "Height: {0:3d} Perr: {1:3d} Ierr: {2:4d}  Derr: {3:4d}".format(height,error,int(iErr),int(dErr))
		
		sleep(dt)
	except KeyboardInterrupt:
		if armDrone:
			print "\nCanary Disarm"
			canary.disarm()
		if logOn:
			data = 'Keyboard Exit\n'
			f.write(data)
			f.close()
		tofBottom.stop_ranging()
		GPIO.output(sensor1_shutdown, GPIO.LOW)
		GPIO.cleanup()
		print "\nKeyboard Exit"
		exit()
# --------------- Test Complete   ---------------------------------------------

if armDrone:
	# ODO: Proper landing sequence
	print "\nCanary Landing..."
	while height>10:
		try:
			# Read time of flight and convert to cm
			distIn = (tofBottom.get_distance())/10
			print distIn
			# Check if in valid range then load into moving average
			if(distIn > 0 and distIn < MAX_DIST_IN):
				distArray.append(distIn)
				del distArray[0]
			height = sum(distArray)/SMA_LENGTH
		except:
			print "Dist error"
		throttle -= 1
		try:
			canary.setThrottle(throttle)
		except:
			print "Write throttle error"
	print "\nCanary Disarm"
	canary.disarm()
f.close()
tofBottom.stop_ranging()
GPIO.output(sensor1_shutdown, GPIO.LOW)
GPIO.cleanup()
if logOn:
	f.close()
print "\nTest Completed"
