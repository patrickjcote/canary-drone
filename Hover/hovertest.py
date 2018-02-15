#Version 0.2
#Authuor: 	Zach Burke
#Date:		12/04/2017
#Modified:
#       Author:         Date:           Note:
#       Patrick Cote    2/08/2018       Add moving average and error handling
#       Patrick Cote    2/09/2018       Add PID Controller and logging
import time
from time import sleep
import RPi.GPIO as GPIO
from CanaryComm import CanaryComm
from UltrasonicSensor import UltrasonicSensor
import serial

# Parameters
droneOn = 0     # enable drone
logOn = 1       # enable data logging
logVerbose = 1  # enable PID logging
setpoint = 25   # set hover height [cm]
SMA_LENGTH = 3  # moving average taps
TMIN = 1450     # Minimum throttle value
TMAX = 1570     # Maximum throttle value
ZMIN = 3        # Minimum valid measured height [cm]
ZMAX = 200      # Maximum valid measured height [cm]
Kt = 50        # Throttle gain [PWM/m]
Kp = 2         # Proportional Gain
Ki = 0          # Integral Gain
Kd = 0          # Derivative Gain
fs = 10         # Sample Rate [samples/sec]


# Init
GPIO.setmode(GPIO.BOARD)
sensor = UltrasonicSensor(32,31)

height = 0
throttle = 0
dt = 0.1 
Zrange = ZMAX-ZMIN
Trange = TMAX-TMIN
distArray = [0]*SMA_LENGTH
dist = 0
dErr = 0
iErr = 0
Zprev = sensor.getDistanceCM()
sleep(dt)

if droneOn:
    canary = CanaryComm(0x08)
    sleep(1)
    canary.arm()
    sleep(1)
    try:
        canary.setThrottle(1500)
    except KeyboardInterrupt:
        canary.disarm()
        exit()




if logOn:
    fname = 'logs/hovertest.'+time.strftime("%Y.%m.%d.%H%M%S")+'.S'+str(setpoint)+'Tl'+str(TMIN)+'Th'+str(TMAX)+'A'+str(SMA_LENGTH)
    fname = fname+'PID'+str(Kp)+'-'+str(Ki)+'-'+str(Kd)+'.log'
    f = open(fname,'a')

while True:
    try:
        try:
            distIn = sensor.getDistanceCM()
        except:
            pass
        # Moving Average Filter
        if(distIn >= ZMIN and distIn <= ZMAX):
            distArray.append(distIn)
            del distArray[0]
            height = sum(distArray)/SMA_LENGTH
        # PID Control
        error = (setpoint-height)
        iErr = iErr + error*dt
        dErr = (height-Zprev)/dt
        Zprev = height
        Tpid = (Kp*error+Ki*iErr-Kd*dErr) * Kt
        # Limit Throttle Values
        throttle = int(min(max(Tpid,TMIN),TMAX))
        # Data Output and Logging - CSV File: 
        # Time, distance measured, filtered height, throttle, Perror, Ierror, Derror, Controller Output,<CR>
        if logOn:
            data = str(time.time())+','+str(distIn)+','+str(height)+','+str(throttle)+','
            if logVerbose:
                data = data + str(error)+','+str(iErr)+','+str(dErr)+','+str(Tpid)+','
            data = data+'\n'
            f.write(data)
        if logVerbose:
            print "distIn: ",distIn," -- height: ",height," cm"
            print "throttle: ",throttle," -- PID Output: ",Tpid
            print "error: P)",error,",I)",iErr,",D)",dErr
        else:
            print "distIn: ",distIn," -- height: ",height," cm"
            print "throttle: ",throttle

        sleep(dt)
        if droneOn:
    	    canary.setThrottle(throttle)
    except KeyboardInterrupt:
        if droneOn:
            print "\nCanary Disarm"
            canary.disarm()
        GPIO.cleanup()
        print "\nKeyboard Exit"
        exit()
GPIO.cleanup()
if logOn:
    f.close()
