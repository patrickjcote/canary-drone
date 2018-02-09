#Version 0.1
#Authuor: 	Zach Burke
#Date:		12/04/2017
#Modified:
#       Author:         Date:           Note:
#       Patrick Cote    2/08/2018       Add moving average and error handling

import time
import RPi.GPIO as GPIO
from time import sleep
from CanaryComm import CanaryComm
#import VL53L0X
#import DigitalIn
from UltrasonicSensor import UltrasonicSensor
import serial

GPIO.setmode(GPIO.BOARD)

sensor = UltrasonicSensor(32,31)
height = 0

setpoint = 25
HOVER_THROTTLE = 1675
THROTTLE_RANGE = 35
SMA_LENGTH = 3
Zmin = 3
Zmax = 200

throttle = 0
#canary = CanaryComm(0x08)
sleep(1)
#canary.arm()
distArray = [0]*SMA_LENGTH
dist = 0

fname = 'logs/height'+str(time.time())+'.S'+str(setpoint)+'T'+str(HOVER_THROTTLE)+'R'+str(THROTTLE_RANGE)+'A'+str(SMA_LENGTH)+'.log'
f = open(fname,'a')
while True:
    try:
        try:
            distIn = sensor.getDistanceCM()
        except:
            pass
	distArray.append(distIn)
	del distArray[0]
	dist = sum(distArray)/SMA_LENGTH
	if(dist >= Zmin and dist <= Zmax):
		height = dist
	if(height < setpoint):
	    throttle = int(HOVER_THROTTLE + THROTTLE_RANGE*((setpoint-height)/(setpoint-Zmin)))
	elif(height > setpoint):
	    throttle = int(HOVER_THROTTLE - THROTTLE_RANGE*((height-setpoint)/(Zmax-setpoint)))
	else:
	    throttle = HOVER_THROTTLE
	data = str(distIn)+','+str(height)+','+str(throttle)+','+str(time.time())+',\n'
	f.write(data)
	print "distIn: ",distIn
	print "height: ",height," cm"
	print "throttle: ",throttle
	sleep(.1)
#	canary.setThrottle(throttle)
    except KeyboardInterrupt:
        print "\nCanary Disarm"
#        canary.disarm()
        GPIO.cleanup()
        print "\nKeyboard Exit"
        exit()
GPIO.cleanup()
f.close()
