import RPi.GPIO as GPIO
from time import sleep


STATUS_LED = 40
BUTTON_PIN = 38

GPIO.setmode(GPIO.BOARD)
GPIO.setup(STATUS_LED, GPIO.OUT)
GPIO.setup(BUTTON_PIN, GPIO.IN)
while True:
	if GPIO.input(BUTTON_PIN):
		GPIO.output(STATUS_LED, GPIO.HIGH)
	else:
		GPIO.output(STATUS_LED, GPIO.LOW)
