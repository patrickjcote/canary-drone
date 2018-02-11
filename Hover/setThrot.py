#Version 0.2
#Date 02/10/2018
#Author: Patrick Cote

# Libraries
import RPi.GPIO as GPIO
import math
from time import sleep, time
from CanaryComm import CanaryComm
from UltrasonicSensor import UltrasonicSensor
from threading import Thread

# Arm the Drone
armDrone = 1

# Parameters
maxThrottle = 1700
minThrottle = 1400
height = 0
throttle = 1000
trange = maxThrottle-minThrottle
dt = .001

# Function for set throttle thread
def setThrottle():
    while True:
        global throttle, canary, height, dist, f
        sensor = UltrasonicSensor(32,31)
        dist = sensor.getDistanceCM()
        if(dist > 4 and dist < 400):
            height = dist
        if throttle:
            throttle = max(min(throttle,maxThrottle),1000)
        else:
            pass
            if armDrone:
                canary.disarm()
        try:
            if armDrone:
                canary.setThrottle(throttle)
        except:
            pass
	data = str(time())+','+str(height)+','+str(throttle)+',\n'
        try:
	    f.write(data)
        except:
            pass
        sleep(.06)

# Init
if armDrone:
    canary = CanaryComm(0x08)
sleep(1)
if armDrone:
    canary.arm()
sleep(1)
fname = 'logs/manualT.'+str(time.time())+'.log'
f = open(fname,'a')
GPIO.setmode(GPIO.BOARD)
t = Thread(target=setThrottle)
t.start()
sleep(.11)

# Forever loop
#   If <ctrl - c>, disarm and quit
try:
    while True:
		print "Height: ",height," cm"
		print "throttle: ",throttle
                tin = input("Set Throttle: ")

                if tin == 0:
                    if armDrone:
                        canary.disarm()
                    exit()
                elif tin == 1: # ramp up
                    for i in range(minThrottle,maxThrottle):
                        throttle = i
                        print throttle
                        sleep(dt)
                elif tin == 2: # ramp down
                    for i in range(maxThrottle,minThrottle,-1):
                        throttle = i
                        print throttle
                        sleep(dt)
                elif tin == 3: # up then down
                    for i in range(minThrottle,maxThrottle):
                        throttle = i
                        print throttle
                        sleep(dt)
                    for i in range(maxThrottle,minThrottle,-1):
                        throttle = i
                        print throttle
                        sleep(dt)
                elif tin == 4: # up then down half speed
                    for i in range(minThrottle,maxThrottle):
                        throttle = i
                        print throttle
                        sleep(dt*2)
                    for i in range(maxThrottle,minThrottle,-1):
                        throttle = i
                        print throttle
                        sleep(dt*2)
                elif tin == 5: # chirp to 10Hz
                    for i in range(0,int(10/dt)):
                        throttle = math.sin(i*i*3.14159*dt)*trange + minThrottle + (trange/2)
                        print throttle
                        sleep(dt)
                else:
                    throttle = tin
    f.close()
    if armDrone:
        canary.disarm()
    GPIO.cleanup()
except KeyboardInterrupt:
    throttle = 0
    canary.disarm()
    GPIO.cleanup()
    exit()
