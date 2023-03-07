import time

import rclpy  # ROS client library
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from utils.tb3_lds_laser import search_object
from utils.tb3_motion import start_search, drive, stop
from utils.tb3_logs import diagnostics


class Tb3(Node):
    def __init__(self):
        super().__init__('tb3')

        self.cmd_vel_pub = self.create_publisher(
                Twist,      # message type
                'cmd_vel',  # topic name
                1)          # history depth

        self.scan_sub = self.create_subscription(
                LaserScan,
                'scan',
                self.scan_callback,  # function to run upon message arrival
                qos_profile_sensor_data)  # allows packet loss

        self.pos = None
        self.orient = [-1, -1, -1]
        self.go = True
        self.front_search = True
        self.back_search = False
        self.right_search = False
        self.left_search = False
        self.ang_vel_percent = 0
        self.lin_vel_percent = 0
        self.object_front = False
        self.min_dist_front = 0.14
        self.min_dist_back = 0.14
        self.min_dist_right = 0.14
        self.min_dist_left = 0.14
        self.state = -2

    def scan_callback(self, msg):
        """ is run whenever a LaserScan msg is received
        """

        search_object(self, laser=msg.ranges)
        start_search(self)

        if self.go:
            drive(self, 15)
            self.go = False
        else:
            if self.object_front:
                stop(self)
        diagnostics(self)

def main(args=None):
    rclpy.init(args=args)

    tb3 = Tb3()
    print('waiting for messages...')


    try:
        rclpy.spin(tb3)  # Execute tb3 node
        # Blocks until the executor (spin) cannot work
    except KeyboardInterrupt:
        pass

    tb3.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
