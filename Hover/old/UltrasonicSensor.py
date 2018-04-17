import time
import RPi.GPIO as GPIO

class UltrasonicSensor():
	def __init__(self, trig, echo):
		GPIO.setwarnings(False)
		self.trig = trig
		self.echo = echo
		GPIO.setup(self.trig, GPIO.OUT)
		GPIO.output(self.trig, False)
		GPIO.setup(self.echo, GPIO.IN)
		time.sleep(.1)
	def getDistanceCM(self):
		GPIO.output(self.trig, True)
		time.sleep(0.00001)
		GPIO.output(self.trig, False)
		while GPIO.input(self.echo)==0:
			start = time.time()
		while GPIO.input(self.echo)==1:
			end = time.time()

		pulse = (end-start)*34300/2
		time.sleep(0.1)
		return pulse


if __name__ == '__main__':
	GPIO.setmode(GPIO.BOARD)
	trig = 32
	echo = 31
	sensor = UltrasonicSensor(trig, echo)
	while True:
		dist = sensor.getDistanceCM()
		if(dist > 5 and dist < 400):
			print "distance: ",dist," cm"
		else:
			print "distance: Out of Range"
	GPIO.cleanup()
