import RPi.GPIO as GPIO
import time

def init():
    GPIO.setmode(GPIO.BCM)
    motor_pins = [23, 25] # BCM pins
    for pin in motor_pins:
        GPIO.setup(pin, GPIO.OUT)

    # Enable pins for two motors
    enable_pins = [13, 12]
    for pin in enable_pins:
            GPIO.setup(pin, GPIO.OUT)
            # GPIO.output(pin, True)

    #or pin in motor_pins:
    #	GPIO.output(pin, True)*/
        #pin.ChangeDutyCycle(100) # (50)% of max speed
        

def forward(sec):
    init()
    motor1 = GPIO.PWM(12, 1000)  # Motor A (pin, pwm freq)
    motor2 = GPIO.PWM(13, 1000)   # Motor B
    motor_pins = [ 23, 25]
    for pin in motor_pins:
        GPIO.output(pin,False)
    motor1.start(0)
    motor2.start(0)
    motor1.ChangeDutyCycle(75)
    motor2.ChangeDutyCycle(75)
    # for pin in motor_pins:
    #     pin.ChangeDutyCycle(50)
    # GPIO.output(22, True) #
    # GPIO.output(23, False)
    # GPIO.output(25, False)
    # GPIO.output(24, True) #
    time.sleep(sec)
    GPIO.cleanup()

# def reverse(sec):
#     init()
#     gpio.output(22, False)
#     gpio.output(23, True)
#     gpio.output(25, False)
#     gpio.output(24, True)
#     time.sleep(sec)
#     gpio.cleanup()


forward(5)
