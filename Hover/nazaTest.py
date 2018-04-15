#Version 0.1
#Authuor: 	Zach Burke
#Date:		12/04/2017

import RPi.GPIO as GPIO
from time import sleep
from CanaryComm import CanaryComm


canary = CanaryComm(0x08)
sleep(1)
canary.arm()
sleep(.5)
canary.setThrottle(1550)
sleep(10)
canary.disarm()
