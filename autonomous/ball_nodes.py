import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from std_msgs.msg import String
import time

class Ball(Node):
    def __init__(self):
        super().__init__('ball_node')
        
        # create subscription to track heat location
        self.launch_ball_subscription = self.create_subscription(
            String,
            'launch_ball',
            self.launch_ball_callback,
            10)
        self.launch_ball_subscription

    def launch_ball_callback(self, msg):
        self.launch_ball_signal = msg.data
        self.get_logger().info(f'Launch Ball Callback got triggered: {msg.data}')
        if self.launch_ball_signal == 'ok':
            print('Lauching the ball!')
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


            time.sleep(2) # Stop for 2 seconds

            servo_pwm.ChangeDutyCycle(10) # Launch the first ball
            time.sleep(1)
            servo_pwm.ChangeDutyCycle(2.5)

            time.sleep(4) # Stop for 4 seconds 

            servo_pwm.ChangeDutyCycle(10) # Launch the second ball
            time.sleep(1)
            servo_pwm.ChangeDutyCycle(2.5)

            time.sleep(2) # Stop for 2 seconds

            servo_pwm.ChangeDutyCycle(10) # Launch the third ball
            time.sleep(1)
            servo_pwm.ChangeDutyCycle(2.5)

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
