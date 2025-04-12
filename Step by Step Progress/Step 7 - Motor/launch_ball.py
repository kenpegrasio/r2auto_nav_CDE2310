import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
motor_pins = [23, 25] # BCM pins
for pin in motor_pins:
    GPIO.setup(pin, GPIO.OUT)

# Enable pins for two motors
enable_pins = [13, 12]
for pin in enable_pins:
    GPIO.setup(pin, GPIO.OUT)
    
motor1 = GPIO.PWM(12, 1000)  # Motor A (pin, pwm freq)
motor2 = GPIO.PWM(13, 1000)   # Motor B
motor_pins = [23, 25]
for pin in motor_pins:
    GPIO.output(pin,False)
motor1.start(0)
motor2.start(0)
motor1.ChangeDutyCycle(75)
motor2.ChangeDutyCycle(75)

servo_pin = 5
GPIO.setup(servo_pin, GPIO.OUT)
servo_pwm = GPIO.PWM(servo_pin, 50)  # 50Hz
servo_pwm.start(2.5)  # Neutral position

servo_pwm.ChangeDutyCycle(10)
time.sleep(1)
servo_pwm.ChangeDutyCycle(2.5)
time.sleep(1)
time.sleep(1)

servo_pwm.ChangeDutyCycle(10)
time.sleep(1)
servo_pwm.ChangeDutyCycle(2.5)
time.sleep(1)
time.sleep(1)

servo_pwm.ChangeDutyCycle(10)
time.sleep(1)
servo_pwm.ChangeDutyCycle(2.5)
time.sleep(1)
time.sleep(1)

GPIO.cleanup()