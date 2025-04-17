import busio
import board
import adafruit_amg88xx
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import qos_profile_sensor_data
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import numpy as np
import time

FRONT_HEAT_LAUNCH = 40
FRONT_HEAT_FORWARD = 34
LEFT_RIGHT_HEAT_THRESHOLD = 30
SHOOTING_AREA_THRESHOLD = 40 * 40

class AMG8833Node(Node):
    def __init__(self):
        super().__init__('amg8833_node')

        # Thermal sensor setup
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.amg = adafruit_amg88xx.AMG88XX(self.i2c)

        # ROS2 publisher
        self.publisher = self.create_publisher(String, '/heat_location', 10)
        self.launch_ball_publisher = self.create_publisher(String, 'launch_ball', 10)

        # create subscription to track occupancy
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])

        self.tfBuffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=5.0))
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.timer = self.create_timer(1.0, self.read_publish_temperature)

        self.shooting_area = set()
        self.last_map_update = None
    
    def occ_callback(self, msg):
        # self.get_logger().info("Received map data!")

        # find transform to obtain base_link coordinates in the map frame
        # lookup_transform(target_frame, source_frame, time)
        try:
            trans = self.tfBuffer.lookup_transform(
                'map', 'base_link', 
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1)
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            print(e)
            return
            
        self.cur_pos = trans.transform.translation

        # get map resolution and map origin 
        self.map_res = msg.info.resolution
        self.map_origin = msg.info.origin.position
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        received_map = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        adjusted_map = np.fliplr(np.rot90(received_map))
        self.cur_map = adjusted_map
        self.last_map_update = time.time()
        self.get_logger().info(f'Updating Map!')

        # get map grid positions for x, y position
        grid_x = round((self.cur_pos.x - self.map_origin.x) / self.map_res) # column in numpy 
        grid_y = round(((self.cur_pos.y - self.map_origin.y) / self.map_res)) # row in numpy
        
        self.currow = self.map_width - 1 - grid_x
        self.curcol = self.map_height - 1 - grid_y

    def point_to_point_distance(self, a, b):
        return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2

    def read_publish_temperature(self):
        if self.last_map_update == None or time.time() - self.last_map_update > 5.0:
            self.get_logger().warn("Map is outdated!")
            return

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

        # Calculate the shortest distance to the shooting areas
        shortest_to_shooting_area = math.inf
        for x, y in self.shooting_area:
            grid_x = round((x - self.map_origin.x) / self.map_res)
            grid_y = round(((y - self.map_origin.y) / self.map_res))
            convertx = self.map_width - 1 - grid_x
            converty = self.map_height - 1 - grid_y
            shortest_to_shooting_area = min(shortest_to_shooting_area, self.point_to_point_distance((convertx, converty), (self.currow, self.curcol)))
        print(f'Shortest distance to shooting area: {shortest_to_shooting_area}')

        if shortest_to_shooting_area > SHOOTING_AREA_THRESHOLD:
            if max(max_element_per_col[2:6]) > FRONT_HEAT_LAUNCH:
                print("Heat is detected in front! Activating servo.")
                self.publisher.publish(String(data='ok'))
                self.launch_ball_publisher.publish(String(data='ok'))
                self.shooting_area.add((self.cur_pos.x, self.cur_pos.y))
                time.sleep(9)
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
        else:
            print('Shoot around this point already!')

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

