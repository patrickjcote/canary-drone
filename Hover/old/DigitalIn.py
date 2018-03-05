import RPi.GPIO as GPIO
from time import sleep

#from interface import implements, Interface
#class Subject(Interface):
#	def registerObserver(): #Observer o):
#		pass
#	def removeObserver(): #Observer o):
#		pass
#	def notifyObservers():
#		pass
#		for observer in self.observers:
#			observer.update()



class DigitalInput(object):
    def __init__(self, pin):
        self.pin = pin
	GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
	GPIO.add_event_detect(self.pin, GPIO.BOTH)
	GPIO.add_event_callback(self.pin, callback=self.edgeTrigger)

    def edgeTrigger(self, pin):
        print('EDGE Detected')
    def regiseterObserver():
	pass
    def removeObserver():
	pass
    def notifyObservers():
	pass



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

    def __init__(self, pin):
        super(IRInput, self).__init__(pin)
	self.inRange = False

    def edgeTrigger(self, pin):
	pass
	#print('Object Detected: %f, %s'%(pin, self.objectInRange()))
	#Observer pattern: notify observers

    def objectInRange(self):
	return not GPIO.input(self.pin)

GPIO.setmode(GPIO.BOARD)

#a = KillSwitchInput(16, 'GO')
#b = KillSwitchInput(18, 'RETURN')
#c = KillSwitchInput(22, 'HOVER')
#d = KillSwitchInput(24, 'KILL')
#e = IRInput(11)
#f = IRInput(13)

#while(True):
#	g = e.objectInRange()
#	h = f.objectInRange()
#	print('In Range a: %s'%g)
#	print('In Range b: %s'%h)
#	print('State: %s'%KillSwitchInput.state)
#	sleep(0.5)
