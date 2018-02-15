#Version 0.2
#Date 02/10/2018
#Author: Patrick Cote

# Libraries
import RPi.GPIO as GPIO
import math
from time import sleep, strftime, time
from CanaryComm import CanaryComm
from UltrasonicSensor import UltrasonicSensor
from threading import Thread

# Arm the Drone
armDrone = 0

# Parameters
maxThrottle = 1700
minThrottle = 1400
maxTrim = 1550
minTrim = 1450
height = 0
throttle = 1000
pitch = 1500
roll = 1500
trange = maxThrottle-minThrottle
dt = .01  # Delay for throttle ramps

# Function for set throttle thread
def setThrottle():
    while True:
        global throttle, canary, height, dist, f, pitch, roll
        if throttle>0:
            if throttle == 1000:
                pass
            else:
                throttle = max(min(throttle,maxThrottle),1000)
        elif throttle<0:
            GPIO.cleanup()
            if armDrone:
                canary.disarm()
                break
        else:
            pass
        pitch = max(min(pitch,maxTrim),minTrim)
        roll = max(min(roll,maxTrim),minTrim)
        if armDrone:
            try:
                canary.setPitch(pitch)
            except:
                print "Set pitch failed"
        if armDrone:
            try:
                canary.setRoll(roll)
            except:
                print "Set roll failed"
        sensor = UltrasonicSensor(32,31)
        try:
            dist = sensor.getDistanceCM()
        except:
            pass
        if(dist > 4 and dist < 400):
            height = dist
        try:
            if armDrone:
                canary.setThrottle(throttle)
        except:
            print "Throttle set error"
        data = str(time())+','+str(height)+','+str(throttle)+','+str(roll)+','+str(pitch)+',\n'
        try:
            f.write(data)
        except:
            print "Data write error"
        sleep(.06)

# Init
if armDrone:
    canary = CanaryComm(0x08)
sleep(1)
if armDrone:
    canary.arm()
sleep(1)
fname = 'logs/trimSet.'+strftime("%Y.%m.%d.%H%M%S")+'.csv'
f = open(fname,'a')
GPIO.setmode(GPIO.BOARD)
t = Thread(target=setThrottle)
t.start()
sleep(.11)

# Forever loop
#   If <ctrl - c>, disarm and quit
try:
    while True:
        # User Input Menu
        print "\n----- Current Status -----"
        print "   Throttle : ",throttle," | Roll: ",roll," | Pitch: ",pitch
        print "\n----- Input Options ------"
        print "   Pitch Up/Down: 1/2 | Roll Left/Right: 3/4 | Throttle Up/Down: 5/6 "
        print "   Disarm: 0 | Exit: -1 or <ctrl>+c"

        try:
            tin = input("\nInput: ")
        except KeyboardInterrupt:
            print "\nDisarming Drone\nKilling Program"
            if armDrone:
                canary.disarm()
            raise
        except:
            tin = 1000
        # 
        if tin == 0:
            if armDrone:
                canary.disarm()
        elif tin == 1:
            print "Pitch Up"
            pitch = pitch + 1
        elif tin == 2:
            print "Pitch Down"
            pitch = pitch - 1
        elif tin == 3:
            print "Roll Right"
            roll = roll + 1
        elif tin == 4:
            print "Roll Left"
            roll = roll - 1
        elif tin == 5:
            print "Increment Throttle"
            throttle = throttle + 1
        elif tin == 6:
            print "Decrement Throttle"
            throttle = throttle - 1
        elif tin < 0:
            f.close()
            if armDrone:
                canary.disarm()
            GPIO.cleanup()
            throttle = tin
            break
        else:
            throttle = tin
    canary.disarm()
    GPIO.cleanup()
    exit()
except KeyboardInterrupt:
    f.close()
    throttle = -1
    canary.disarm()
    GPIO.cleanup()
    exit()
