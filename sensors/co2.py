import RPi.GPIO as GPIO
import serial
from time import sleep

CO2_PIN = 40

ser = serial.Serial("/dev/ttyAMA0")

GPIO.setmode(GPIO.BOARD)
GPIO.setup(CO2_PIN, GPIO.OUT)
GPIO.output(CO2_PIN, GPIO.LOW)

ser.write("K 1\r\n")
sleep(.05)

while True:
	line = ser.readline()
	outStr = "CO2 Concentration: "+str(line[3:6])+"."+str(line[6:8])+"%"
	conc = int(line[3:8])
	
	if conc > 1000:
		GPIO.output(CO2_PIN, GPIO.HIGH)
	else:
		GPIO.output(CO2_PIN, GPIO.LOW)
	print outStr
