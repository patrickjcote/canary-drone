# File: kevenDemoNazaCO2.py
# Version: 0.5
# Author: 2018 - Patrick Cote
# Description: Hover Demo for Kevin with CO2 sensor

# --------------- Test Settings------------------------------------------------
logOn = 1		# enable data logging
FILE_DIR = 'logs/kevinDemo2/'	# Log file directory
testDur = 15		# Length of test [s]
setpoint = 55	# [cm]
SETPOINT2 = 55	# [cm] step change @ u[t-testDur/2]
TAKEOFF_PITCH = 1500	# Pitch value to combat ugly takeoff
TAKE_OFF_SET = 50	# [cm] min takeoff height
#  Flight Value Limits
TMAX = 1575		# Max throttle value
TMIN = 1425		# Min throttle value
TMID = 1500		# Initial Throttle
# Time of Flight Mode 0-good,1-better,2-Best,3-Long Range,4-High Speed
TOF_MODE_B = 3	# Ranging mode for downward-facing ToF
TOF_MODE = 3	# Ranging mode of azimuth ToFs
MAX_DIST_IN = 300 # Maximum valid distance read by ToFs
SMA_LENGTH = 3	# Length of Simple Moving Average
# Controller Values
KP_ALT = 1		# Proportional Gain for the altitude controller

# -----------------------------------------------------------------------------

# Import Dependencies
import sys
import RPi.GPIO as GPIO
import VL53L0X
from time import sleep, strftime, time
from CanaryComm import CanaryComm
from threading import Thread
from Co2Comm import CO2Comm

# Check for input arguments
#if len(sys.arg)>0:
#	userInputFlag = sys.arg[0]
#else:
#	userInputFlag = 1

userInputFlag = 0

if userInputFlag:
# ------- User Input ----------------------------------------------------------
	armDrone = input("Arm drone [0 - No, 1 - yes]: ")	 # enable drone

# Controller Gains
	Kp = KP_ALT
	Ki = 0
	Kd = 0
	IERR_LIM = 70	# Max +/- integral error for windup reset
#Kp = input("Kp Gain : ")		# Proportional gain
#Ki = input("Ki Gain : ")		# Integral gain
#Kd = input("Kd Gain : ")		# Derivative gain
# Display Test Parameters
	print "---- Test Plan ----"
	print "Set point: ",setpoint
	print "Throttle Range of ",TMIN,"-",TMAX," for ",testDur,"s"
	print "P: ",Kp," I: ",Ki," Kd: ",Kd

# Confirm Drone Arming
	if armDrone:
		armDrone = input("Confirm drone arm [0 - No, 1 - yes]: ")
# ---- End User Input ---------------------------------------------------------
else:
	# Controller Gains
	Kp = KP_ALT
	Ki = 0
	Kd = 0
	IERR_LIM = 70	# Max +/- integral error for windup reset
	armDrone = 1

# ----------------------------Initialization ---------------------------------
# -- Status Light and Start Button --
STATUS_LED = 12
BUTTON_PIN = 40

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(STATUS_LED, GPIO.OUT)
GPIO.setup(BUTTON_PIN, GPIO.IN)

print "Press the 'A' button on the Kill Switch to initiate takeoff..."
# Wait for button to init test, Flash LED to signal ready
while not GPIO.input(BUTTON_PIN):
	GPIO.output(STATUS_LED, GPIO.HIGH)
	sleep(.5)
	GPIO.output(STATUS_LED, GPIO.LOW)
	sleep(.5)
# Rapid flash Status LED to signal takeoff has been enabled
# Turn off the LED for 1 second then blink once to indicate takeoff
for i in range(0,10):
	GPIO.output(STATUS_LED, GPIO.HIGH)
	sleep(.1)
	GPIO.output(STATUS_LED, GPIO.LOW)
	sleep(.1)
sleep(1)
GPIO.output(STATUS_LED, GPIO.HIGH)
sleep(.1)
GPIO.output(STATUS_LED, GPIO.LOW)
sleep(.1)

# -- ToF Sensor Initialization --
# GPIO for Sensor 1 shutdown pin
sensor1_shutdown = 31 # DOWN
# GPIO for Sensor 2 shutdown pin
sensor2_shutdown = 32 # FWD

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
sleep(1)

# Test ToFs Readings
if tofFront.get_distance() < 0:
	print "ToF Error: Front reads -1"
	print "Program Exit"
	exit()
if tofBottom.get_distance() < 0:
	print "ToF Error: Bottom reads -1"
	print "Program Exit"
	exit()

# ----------------------- Start Main Program ----------------------------------
if armDrone:
	# Init Canary Class
	canary = CanaryComm(0x08)
	# Test Connection to Cypress
	try:
		cypressConnection = canary.disarm()
		while cypressConnection == 0:
			print "Cypress not detected."
			print "Press the 'D' button on the Kill Switch..."
			cypressConnection = canary.disarm()
			sleep(0.5)
	except:
		pass
	print "Cypress Detected!"

if logOn:
	try:
		fname = FILE_DIR
		if armDrone == 0:
			fname = fname+'x'
		fname = fname+strftime("%Y.%m.%d.%H%M%S")+'.S'+str(setpoint)
		fname = fname+'Tl'+str(TMIN)+'Th'+str(TMAX)+'A'
		fname = fname+'PID'+str(Kp)+'-'+str(Ki)+'-'+str(Kd)+'.csv'
		f = open(fname,'a')
	except:
		print "Log File Error..."
		raise
		exit()

print "Press the 'A' button on the Kill Switch to initiate takeoff..."
# Wait for button to init test, Flash LED to signal ready
while not GPIO.input(BUTTON_PIN):
	GPIO.output(STATUS_LED, GPIO.HIGH)
	sleep(.5)
	GPIO.output(STATUS_LED, GPIO.LOW)
	sleep(.5)
# Rapid flash Status LED to signal takeoff has been enabled
# Turn off the LED for 1 second then blink once to indicate takeoff
for i in range(0,10):
	GPIO.output(STATUS_LED, GPIO.HIGH)
	sleep(.1)
	GPIO.output(STATUS_LED, GPIO.LOW)
	sleep(.1)
sleep(1)
GPIO.output(STATUS_LED, GPIO.HIGH)
sleep(.1)
GPIO.output(STATUS_LED, GPIO.LOW)
sleep(.1)

# Init the CO2 detection w/ printing and logging
co2Dir = FILE_DIR
if armDrone == 0:
	co2Dir = co2Dir + 'x'
co2 = CO2Comm(STATUS_LED,1,1,co2Dir)
sleep(.1)

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
		while(height<TAKE_OFF_SET):
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
# Main loop:
# Remain in loop for length of testDur or until "Go Button" is pressed
# Logic:
#	Read ToF, filter measurement to get distance to the ground
#	Calculate error and Calculate new throttle value
#	Log values and display them to the console
#	Delay for ToF sampling time
tstart = time()
while time()<(tstart+testDur) and (not GPIO.input(BUTTON_PIN)):
	try:
		if time()>(tstart+testDur/2):
			setpoint = SETPOINT2
	# ---------  Altitude Hold Controller
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
		# Calculate Error
		error = (setpoint-height)
		iErr = iErr + error*dt
		# Intergral Anti-windup reset
		iErr = min(max(iErr,IMIN),IMAX)
		dErr = (height-heightPrev)/dt
		heightPrev = height
		Tpid = Kp*error+Ki*iErr-Kd*dErr + TMID
		# Limit Controller output
		throttle = int(min(max(Tpid,TMIN),TMAX))
		# Update Throttle
		if armDrone:
			try:
				canary.setThrottle(throttle)
			except:
				raise
				print "Throttle set error"

	# --------  Logging - CSV File: -----------------------------
		# Time, height, throttle, error, controller out, <CR>
		if logOn:
			data = str(time())+','+str(height)+','+str(throttle)+','
			data = data + str(error)+','+str(Tpid)+','+str(setpoint)+','
			data = data+'\n'
			f.write(data)

	# --------- Status Output -----------------------------------
		print "Height: {0:3d}cm  Error: {1:3d} Throttle: {2:4d}".format(height,error,throttle)
		sleep(dt)

	except KeyboardInterrupt:
		if armDrone:
			print "\nCanary Disarm"
			canary.disarm()
		if logOn:
			data = 'Keyboard Exit\n'
			f.write(data)
			f.close()
		tofFront.stop_ranging()
		tofBottom.stop_ranging()
		co2.exit()
		GPIO.output(sensor1_shutdown, GPIO.LOW)
		GPIO.cleanup()
		print "\nKeyboard Exit"
		exit()
# --------------- Test Complete   ---------------------------------------------
if GPIO.input(BUTTON_PIN):
	print "KILL SWITCH ENGAGE!"

if armDrone:
	# TODO: Proper landing sequence
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
			print "Landing Throttle:",throttle
		except:
			print "Write throttle error"
	print "\nCanary Disarm"
	sleep(.5)
	try:
		canary.setThrottle(1001)
		sleep(.2)
		canary.disarm()
		sleep(.2)
		canary.close()
	except:
		print "Disarm Failed... RUN!"
f.close()
co2.exit()
sleep(.1)
tofBottom.stop_ranging()
tofFront.stop_ranging()
GPIO.output(sensor1_shutdown, GPIO.LOW)
GPIO.cleanup()
if logOn:
	f.close()
print "\nTest Completed"
