import RPi.GPIO as GPIO
import serial
from time import sleep
from threading import Thread
  
GPIO.setmode(GPIO.BOARD)
GPIO.setup(12, GPIO.OUT)
GPIO.output(12, GPIO.LOW)
while True:
	GPIO.output(12, GPIO.HIGH)
	sleep(.2)
	GPIO.output(12, GPIO.LOW)
	sleep(.2)
