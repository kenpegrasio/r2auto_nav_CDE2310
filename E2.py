import time
import sys  # Import sys to exit the program
import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

# Set pin numbering convention
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        print("Listener node initialized")

        # Set QoS policy to "Best Effort"
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Match LiDAR QoS
            depth=10
        )

        self.subscription = self.create_subscription(
            LaserScan, 
            '/scan',
            self.callback,
            qos_profile)
        
        print("Subscribed to /scan with BEST EFFORT QoS")

    def callback(self, msg):
        print("Received scan data")
        print(msg.ranges[-1])  # Print the actual LiDAR distance data
        if msg.ranges[-1] <= 0.5:
            print('Near Object Detected')
            print('Setting servo')
            p.ChangeDutyCycle(5)
            time.sleep(1)
            print('Setting solenoid')
            trigger_solenoid()
            
            # Stop the program completely
            self.terminate_program()

    def terminate_program(self):
        print("Terminating program...")
        self.destroy()
        rclpy.shutdown()
        sys.exit(0)  # Forcefully exit the script

    def destroy(self):
        print("Cleaning up GPIO and shutting down node...")
        p.stop()  # Stop PWM
        GPIO.cleanup()  # Clean up GPIO
        self.destroy_node()  # Destroy ROS2 node

# Define the servo pin
servo_pin = 12
GPIO.setup(servo_pin, GPIO.OUT)

# Set PWM frequency to 50Hz
p = GPIO.PWM(servo_pin, 50)

# Start PWM with neutral position (90 degrees)
p.start(7.5)

# Function to convert angle to duty cycle
def set_angle(angle):
    duty_cycle = 2.5 + (angle / 180.0) * 10  # Maps 0-180 degrees to 2.5-12.5 duty cycle
    p.ChangeDutyCycle(duty_cycle)

# Define the solenoid pin
solenoid_pin = 40
GPIO.setup(solenoid_pin, GPIO.OUT)

# Function to trigger the solenoid
def trigger_solenoid():
    GPIO.output(solenoid_pin, 1)

set_angle(0)
GPIO.output(solenoid_pin, 0)
rclpy.init()
node = Listener()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
