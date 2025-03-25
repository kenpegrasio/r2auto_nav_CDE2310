import time
import busio
import board
import adafruit_amg88xx
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class AMG8833Node(Node):
    def __init__(self):
        super().__init__('amg8833_node')

        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.amg = adafruit_amg88xx.AMG88XX(self.i2c)

        self.publisher = self.create_publisher(Float32MultiArray, '/temperature_map', 10)
        self.timer = self.create_timer(1, self.read_publish_temperature)
    
    def read_publish_temperature(self):
        temperature_data = Float32MultiArray()
        
        # Flatten the 2D list of pixels into a 1D list of temperature values
        temperature_data.data = [temp for row in self.amg.pixels for temp in row]
        
        # Publish the temperature data
        self.publisher.publish(temperature_data)

        # Optionally print the temperature data for debugging
        for row in self.amg.pixels:
            print(["{0:.1f}".format(temp) for temp in row])
        print("\n")


def main(args=None):
    rclpy.init(args=args)
    node = AMG8833Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()