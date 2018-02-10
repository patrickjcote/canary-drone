#Version 0.1
#Date 02/09/2018
#Author: Zach Burke

import RPi.GPIO as GPIO
from time import sleep
import time
from CanaryComm import CanaryComm
from UltrasonicSensor import UltrasonicSensor
from threading import Thread

GPIO.setmode(GPIO.BOARD)

sensor = UltrasonicSensor(32,31)
height = 0

armDrone = 0
maxThrottle = 1700
minThrottle = 1400
throttle = 1000
dt = .001

def setThrottle():
    while True:
        global throttle
        global canary
        global f
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
	data = str(time.time())+','+str(height)+','+str(throttle)+',\n'
        try:
	    f.write(data)
        except:
            pass
        sleep(.1)


if armDrone:
    canary = CanaryComm(0x08)
sleep(1)
if armDrone:
    canary.arm()
sleep(1)

fname = 'logs/manualT.'+str(time.time())+'.log'
f = open(fname,'a')

t = Thread(target=setThrottle)
t.start()

try:
    while True:
		dist = sensor.getDistanceCM()
		if(dist > 4 and dist < 400):
			height = dist
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
                elif tin == 4: # up then down
                    for i in range(minThrottle,maxThrottle):
                        throttle = i
                        print throttle
                        sleep(dt*2)
                    for i in range(maxThrottle,minThrottle,-1):
                        throttle = i
                        print throttle
                        sleep(dt*2)

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
