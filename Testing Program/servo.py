# This code is used for testing the servo

import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)

# Define the servo pin
servo_pin = 12
GPIO.setup(servo_pin, GPIO.OUT)

p = GPIO.PWM(servo_pin, 50)
p.start(7.5)

while True:
	print('0 degree')
	p.ChangeDutyCycle(2.5)
	time.sleep(1)
	print('180 degree')
	p.ChangeDutyCycle(12.5)
	time.sleep(1)
