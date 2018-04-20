# File: fwdAvoid.py
# Version: 0.2
# Author: 2018 - Patrick Cote
# Description: Hover Controller with forward object detection
#TODO: test
# --------------- Test Settings------------------------------------------------
logOn = 1		# enable data logging
setpoint = 75	# [cm]
SETPOINT2 = 75	# [cm]   step change in set point halfway through the test
TAKEOFF_PITCH = 1520	# Pitch value to combat ugly takeoff
testDur = 20	# Length of test [s]
# Flight Control Limits
TMAX = 1575		# Max throttle value
TMIN = 1425		# Min throttle value
TMID = 1500		# Initial Throttle
PMIN = 1450		# Min Pitch Value
PMAX = 1550		# Max Pitch Value
PSTEP = 5		# Increment Pitch with distance array multiplier
# Distance Array [Max, Mid, Min, Kill]
SENSE_DIST = [150, 75, 50, 25]
SMA_LENGTH = 3	# Length of Simple Moving Average for altitude ToF
SMA2 = 9		# Length of Simple Moving Average for azimuth ToFs
# Time of Flight Mode 0-good,1-better,2-Best,3-Long,4-High Speed
TOF_MODE = 3
TOF_MODE_B = 4
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

# Init Classes
canary = CanaryComm(0x08)


#-------------ToF Sensor Initialization-----------------------------------
# GPIO for Sensor 1 shutdown pin
sensor1_shutdown = 7 #DOWN
# GPIO for Sensor 2 shutdown pin
sensor2_shutdown = 36 #FWD

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

#--------------------------------------------------------------
sleep(0.50)
timing = tofBottom.get_timing()
if (timing < 20000):
    timing = 20000
dt = timing/1000000.00
#End ToF sensor initialization--------------------------------------------------

# ------- User Input ----------------------------------------------------------
armDrone = input("Arm drone [0 - No, 1 - yes]: ")	 # enable drone

# Controller Gains
Kp = 2
Ki = 0
Kd = 0
IERR_LIM = 70	# Max +/- integral error for windup reset
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
GPIO.output(STATUS_LED, GPIO.HIGH)
sleep(.2)
GPIO.output(STATUS_LED, GPIO.LOW)

# --------------- Takeoff Sequence---------------------------------------------
height = 0
distArray = [0]*SMA_LENGTH
fwdDistArray = [MAX_DIST_IN]*SMA2
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
fwdDist = MAX_DIST_IN
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

allClearFlag = 1
while time()<(tstart+testDur) and allClearFlag:
	try:
		# -----  Step Input Testing --------
		if time()>(tstart+testDur/2):
			setpoint = SETPOINT2

		# -- Altitude Hold Controller ------
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


		#------ Front/Back Controller --- Pitch Control
		# Assume we are clear and level pitch
		pitchSet = 1500
		# If we aren't clear adjust the pitch
		if fwdDist < MAX_DIST_IN:
			if fwdDist < SENSE_DIST[0]:
				pitchSet = 1500 - PSTEP
			if fwdDist < SENSE_DIST[1]:
				pitchSet = 1500 - 2*PSTEP
			if fwdDist < SENSE_DIST[2]:
				pitchSet = 1500 - 3*PSTEP
			if fwdDist < SENSE_DIST[3]:
				print "YOU'RE TOO CLOSE, MAN!!!!"
#				allClearFlag = 0
		# Limit Pitch Values
		pitch = int(min(max(pitchSet,PMIN),PMAX))
		
		# Update Throttle
		try:
			canary.setPitch(pitch)
			print "Pitch: ",pitch
		except:
			raise
			print "Pitch set error"
		# Update Distance
		try:
			# Read time of flight and convert to cm
			fwdIn = (tofFront.get_distance())/10
			print "Fwd Read: ",fwdIn
			# Check if in valid range then load into moving average
			if(fwdIn > 0 and fwdIn < MAX_DIST_IN):
				fwdDistArray.append(fwdIn)
				del fwdDistArray[0]
			else:
				fwdDistArray.append(MAX_DIST_IN)
				del fwdDistArray[0]
			fwdDist = sum(fwdDistArray)/SMA2
			print "Fwd SMA: ",fwdDist
		except:
			print "Fwd Dist error"



		#------------ Logging - CSV File: --------------------------------
		# Time, height, throttle, error, throttle, setpoint, pitch <CR>
		if logOn:
			data = str(time())+','+str(height)+','+str(throttle)+','
			data = data + str(error)+','+str(Tpid)+','
			data = data+str(pitch)+','+str(setpoint)+'\n'
			f.write(data)

		#-------------- Status Output
		print "Height: ",height,"cm  |   Throttle: ",throttle
		
		sleep(dt)
	except KeyboardInterrupt:
		if armDrone:
			print "\nCanary Disarm"
			canary.disarm()
		if logOn:
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
	tTakeoff = time()
	while time()<(tTakeoff + 2):
		canary.setThrottle(1450)
	while time()<(tTakeoff + 3.5):
		canary.setThrottle(1400)
	print "\nCanary Disarm"
	canary.disarm()
if allClearFlag == 0:
	print "Something got too close"
f.close()
tofBottom.stop_ranging()
GPIO.output(sensor1_shutdown, GPIO.LOW)
GPIO.cleanup()
if logOn:
	f.close()
print "\nTest Completed"
