#Version 0.1
#Authuor: 	Zach Burke
#Date:		12/03/2017

import RPi.GPIO as GPIO
from time import sleep

class DigitalInput(object):
    def __init__(self, pin):
        self.pin = pin
	GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
	GPIO.add_event_detect(self.pin, GPIO.BOTH)
	GPIO.add_event_callback(self.pin, callback=self.edgeTrigger)

    def edgeTrigger(self, pin):
        print('EDGE Detected')


class KillSwitchInput(DigitalInput):
    state = 'IDLE'

    def __init__(self, pin, state):
        super(KillSwitchInput, self).__init__(pin)
	self.state = state

    def edgeTrigger(self, pin):
        KillSwitchInput.state = self.state

    def registerObserver():
	pass

class IRInput(DigitalInput):

    def __init__(self, pin, heading):
        super(IRInput, self).__init__(pin)
	self.inRange = False
	self.heading = heading

    def edgeTrigger(self, pin):
	pass
	#self.inRange = not GPIO.input(self.pin)
	#print('IR %f in Range: %s'%(self.pin, self.inRange))

    def objectInRange(self):
	return not GPIO.input(self.pin)

GPIO.setmode(GPIO.BOARD)
sensors = [IRInput(11, 0), IRInput(13, 90)]

while True:
    headings = {0 : True, 90 : True, 180 : True, 270 : True}
    for s in sensors:
	if s.objectInRange():
	    headings[s.heading] = False
#	    print(headings[s.heading])
    	    #headings[None] = False
    for h in headings:
	if(headings[h]):
	    print(h)
		#call heading command
	    break;
    sleep(0.01)
