import busio
import board
import adafruit_amg88xx
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

FRONT_HEAT_LAUNCH = 37
FRONT_HEAT_FORWARD = 32
LEFT_RIGHT_HEAT_THRESHOLD = 33

class AMG8833Node(Node):
	def __init__(self):
		super().__init__('amg8833_node')

		# Thermal sensor setup
		self.i2c = busio.I2C(board.SCL, board.SDA)
		self.amg = adafruit_amg88xx.AMG88XX(self.i2c)

		# ROS2 publisher
		self.publisher = self.create_publisher(String, '/heat_location', 10)
		self.timer = self.create_timer(1.0, self.read_publish_temperature)

	def read_publish_temperature(self):
		# Debug print
		for row in self.amg.pixels:
			print(["{0:.1f}".format(temp) for temp in row])
		print("\n")
		
		print("Max element per column")
		max_element_per_col = []
		for idxcol in range(8):
			maxcurcol = -1
			for idxrow in range(8):
				maxcurcol = max(maxcurcol, self.amg.pixels[idxrow][idxcol])
			print(maxcurcol, end=' ')
			max_element_per_col.append(maxcurcol)
		print()

		if max(max_element_per_col[2:6]) > FRONT_HEAT_LAUNCH:
			print("Heat is detected in front! Activating servo.")
			self.publisher.publish(String(data='ok'))
		elif max(max_element_per_col[2:6]) > FRONT_HEAT_FORWARD:
			print("Heat is detected in front. Move closer please!")
			self.publisher.publish(String(data='forward'))
		elif max(max_element_per_col[:2]) > LEFT_RIGHT_HEAT_THRESHOLD:
			print("Heat is on the Right")
			self.publisher.publish(String(data='right'))
		elif max(max_element_per_col[6:]) > LEFT_RIGHT_HEAT_THRESHOLD:
			print("Heat is on the Left")
			self.publisher.publish(String(data='left'))
		else:
			print("No heat ahead. Continue Exploration.")
			self.publisher.publish(String(data='null'))

	def destroy_node(self):
		super().destroy_node()

def main(args=None):
	rclpy.init(args=args)
	node = AMG8833Node()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()

