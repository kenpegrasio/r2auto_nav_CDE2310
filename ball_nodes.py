import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from std_msgs.msg import String
import time

class Ball(Node):
    def __init__(self):
        super().__init__('ball_node')
        
        # create subscription to track heat location
        self.heat_subscription = self.create_subscription(
            String,
            'heat_location',
            self.heat_callback,
            10)
        self.heat_subscription

    def heat_callback(self, msg):
        self.heat_location = msg.data
        self.get_logger().info(f'Heat Callback got triggered: {msg.data}')
        if self.heat_location == 'ok':
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
            servo_pwm.stop()
    
    def destroy_node(self):
        super().destroy_node()

def main(args=None):
	rclpy.init(args=args)
	node = Ball()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()
