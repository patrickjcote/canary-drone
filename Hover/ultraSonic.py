import time
import RPi.GPIO as GPIO

class UltrasonicSensor():
	def __init__(self, trig, echo):
		self.trig = trig
		self.echo = echo
		GPIO.setup(self.trig, GPIO.OUT)
		GPIO.output(self.trig, False)
		GPIO.setup(self.echo, GPIO.IN)
		time.sleep(.1)
	def getDistanceCM(self):
		time.sleep(.01)
		GPIO.output(self.trig, True)
		time.sleep(0.00001)
        	GPIO.output(self.trig, False)
		start = time.time()
	        while GPIO.input(self.echo)==0:
        	        start = time.time()
        	while GPIO.input(self.echo)==1:
                	end = time.time()

	        pulse = (end-start)*34300/2
		return pulse


if __name__ == '__main__':
	GPIO.setmode(GPIO.BOARD)
	GPIO.setwarnings(False)
	trig = 32
	echo = 31
	trig1 = 40 
	echo1 = 37
	sensor = UltrasonicSensor(trig, echo)
	sensor1 = UltrasonicSensor(trig1, echo1)
	while True:
		dist = sensor.getDistanceCM()
		dist1 = sensor1.getDistanceCM()
		if(dist > 5 and dist < 400):
			print "distance: ",dist," cm"
		else:
			print "distance: Out of Range"
		if(dist1 > 5 and dist1 < 400):
			print "distance1: ",dist1," cm"
		else:
			print "distance1: Out of Range"
	GPIO.cleanup()
