import os
import time

import rclpy  # ROS client library
from rclpy.node import Node as Botnode
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from utils.tb3_logs import diagnostics
from utils.tb3_motion import drive, rotate, stop


# states
# 0: normal
# 1: on the front wall
class Tb3(Botnode):
    def __init__(self):
        super().__init__('tb3')

        self.cmd_vel_pub = self.create_publisher(
            Twist,  # message type
            'cmd_vel',  # topic name
            1)  # history depth

        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,  # function to run upon message arrival
            qos_profile_sensor_data)  # allows packet loss

        self.pos = None
        self.orient = [-1, -1, -1]
        self.state = 0
        self.go = True
        self.front_search = True
        self.back_search = False
        self.right_search = False
        self.left_search = False
        self.object_front = False
        self.object_back = False
        self.object_left = False
        self.object_right = False
        self.ang_vel_percent = 0
        self.lin_vel_percent = 0

    def scan_callback(self, msg):
        """
        is run whenever a LaserScan msg is received
        """
        min_dist_front = 0.32
        min_dist_right = 0.255

        if self.state == 0:
            # Drive to wall
            drive(self, 30)
            if min_dist_front >= msg.ranges[0] > 0:
                self.state = 1
                stop(self)
        elif self.state == 1:
            # Rotate 90 degrees
            rotate(self, 10)
            if min_dist_right >= msg.ranges[-90] > 0:
                self.state = 2
                stop(self)
        elif self.state == 2:
            # Drive to final wall
            min_dist_front = 0.25
            drive(self, 30)
            if min_dist_front >= msg.ranges[0] > 0:
                stop(self)
                self.state = 3
        else:
            pass
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
