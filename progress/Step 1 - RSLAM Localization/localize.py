# This code require 
# ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
# to be run beforehand

# The expected behaviour is that you will find "Received map data!" and "Map is saved!" 
# being log, then you would find map_data.txt where 97 is the robot position

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

# Convert quaternion to Euler angles
def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = max(-1.0, min(+1.0, t2))
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z

class Localization(Node):
    def __init__(self):
        super().__init__('map_odom_listener')

        self.map_subscription = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)

        '''
        self.odom_subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        '''

        self.tfBuffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=5.0))
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.map_metadata = None 
        self.get_logger().info("Localization node started.")

    def map_callback(self, msg):
        self.get_logger().info("Received map data!")

        # find transform to obtain base_link coordinates in the map frame
        # lookup_transform(target_frame, source_frame, time)
        try:
            trans = self.tfBuffer.lookup_transform(
                'map', 'base_link', 
                rclpy.time.Time().to_msg(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            print(e)
            return
            
        cur_pos = trans.transform.translation
        self.x = cur_pos.x
        self.y = cur_pos.y
        
        # convert quaternion to Euler angles
        # cur_rot = trans.transform.rotation
        # roll, pitch, yaw = euler_from_quaternion(cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)
        # self.roll,self.pitch,self.yaw = roll,pitch,yaw
        # self.get_logger().info('Rot-Yaw: R: %f D: %f' % (yaw, np.degrees(yaw)))

        # get map resolution and map origin 
        map_res = msg.info.resolution
        map_origin = msg.info.origin.position

        # get map grid positions for x, y position
        grid_x = round((self.x - map_origin.x) / map_res) # column in numpy 
        grid_y = round(((self.y - map_origin.y) / map_res)) # row in numpy
        
        # save the map to map_data.txt 
        # note: we put indicate 97 as the current robot position
        received_map = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        received_map[grid_y][grid_x] = '97'
        final_map = np.fliplr(np.rot90(received_map))
        np.savetxt("map_data.txt", final_map, fmt="%2d")
        self.get_logger().info("Map is saved!")
        # self.get_logger().info('Grid Y: %i Grid X: %i' % (grid_y, grid_x))

        '''
        def odom_callback(self, msg):
            self.odom_data = msg
            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y
            self.yaw = euler_from_quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
            self.get_logger().info("x y " + str(self.x) + " " + str(self.y))
        '''


def main(args=None):
    rclpy.init(args=args)
    node = Localization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down map & odom listener node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
