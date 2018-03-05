#Version 0.1
#Date 02/09/2018
#Author: Zach Burke

import RPi.GPIO as GPIO
from time import sleep
import time
from CanaryComm import CanaryComm
from UltrasonicSensor import UltrasonicSensor

GPIO.setmode(GPIO.BOARD)

sensor = UltrasonicSensor(32,31)
height = 0

setpoint = 70
throttle = 1500
dThrot = 7
maxThrottle = 1700
minThrottle = 1400

canary = CanaryComm(0x08)
sleep(1)
canary.arm()
sleep(1)

fname = 'logs/tup.'+str(time.time())+'.SP.'+str(setpoint)+'.log'
f = open(fname,'a')

try:
    while True:
		dist = sensor.getDistanceCM()
		if(dist > 4 and dist < 400):
			height = dist

		if(height < setpoint):
			if(throttle < maxThrottle):
				throttle+=dThrot
		else:
			print "Reached setpoint, disarming"
			break
		data = str(time.time())+','+str(height)+','+str(throttle)+',\n'
                try:
		    f.write(data)
                except:
                    pass
		print "Height: ",height," cm"
		print "throttle: ",throttle
                try:
                    canary.setThrottle(throttle)
                except:
                    pass
    sleep(1)
    while True:
        dist = sensor.getDistanceCM()
	if(dist >4 and dist < 400):
	    height = dist
	if(height < 8):
	    break
	else:
	    if(throttle > minThrottle):
	        throttle -= dThrot
	    print "Height: ",height," cm"
            print "throttle: ",throttle
	    data = str(time.time())+','+str(height)+','+str(throttle)+',\n'
	    data = data+'\n'
	    f.write(data)
            try:
                canary.setThrottle(throttle)
            except:
                pass
    f.close()	
    canary.disarm()
    GPIO.cleanup()
except KeyboardInterrupt:
    canary.disarm()
    GPIO.cleanup()
    exit()
