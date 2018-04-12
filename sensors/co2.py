import serial
from time import sleep

ser = serial.Serial("/dev/ttyAMA0")

ser.write("K 1\r\n")
sleep(.05)

while True:
	line = ser.readline()
	outStr = "CO2 Concentration: "+str(line[3:6])+"."+str(line[6:8])+"%"
	# print line[0:9]
	print outStr
