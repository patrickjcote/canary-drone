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
armDrone = 1

# Parameters
maxThrottle = 1700
minThrottle = 1400
height = 0
throttle = 1000
trange = maxThrottle-minThrottle
dt = .01  # Delay for throttle ramps

# Function for set throttle thread
def setThrottle():
    while True:
        global throttle, canary, height, dist, f
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
        data = str(time())+','+str(height)+','+str(throttle)+',\n'
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
fname = 'logs/setThrot.'+strftime("%Y.%m.%d.%H%M%S")+'.csv'
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
        print "   Height   : ",height," cm"
        print "   Throttle : ",throttle
        print "\n----- Input Options ------"
        print "   Sinusoid: f=0-9 -- 5 Second Sine wave of frequency f<9Hz"
        print "   Ramp: 10-Up, 20-Down, 30-Up/Down, 40-Up/Down (1/2 speed) , 50-Chirp (0-8Hz)"
        print "   Set Throttle: ",minThrottle," to ", maxThrottle
        print "   Disarm: 0"
        print "   Exit: -1 or <ctrl>+c"

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
        elif tin == 10: # ramp up
            for i in range(minThrottle,maxThrottle):
                throttle = i
                print "Throttle Ramping: ",throttle
                sleep(dt)
            throttle = 1000
        elif tin == 20: # ramp down
            for i in range(maxThrottle,minThrottle,-1):
                throttle = i
                print "Throttle Ramping: ",throttle
                sleep(dt)
            throttle = 1000
        elif tin == 30: # up then down
            for i in range(minThrottle,maxThrottle):
                throttle = i
                print "Throttle Ramping: ",throttle
                sleep(dt)
            for i in range(maxThrottle,minThrottle,-1):
                throttle = i
                print "Throttle Ramping: ",throttle
                sleep(dt)
            throttle = 1000
        elif tin == 40: # up then down half speed
            for i in range(minThrottle,maxThrottle):
                throttle = i
                print "Throttle Ramping: ",throttle
                sleep(dt*2)
            for i in range(maxThrottle,minThrottle,-1):
                throttle = i
                print "Throttle Ramping: ",throttle
                sleep(dt*2)
            throttle = 1000
        elif tin == 50: # chirp to 8Hz
            for i in range(0,int(8/dt)):
                throttle = int(math.sin(i*i*3.14159*dt)*trange/2) + minThrottle + int(trange/2)
                print "Throttle Ramping: ",throttle
                sleep(dt)
            throttle = 1000
        elif 0 < tin and tin < 10: # Sinusoid
            for i in range(0,int(5/dt)):
                throttle = int(math.sin(tin*2*3.14159*i*dt)*trange/2) + minThrottle + int(trange/2)
                print "Throttle Ramping: ",throttle
                sleep(dt)
            throttle = 1000
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
