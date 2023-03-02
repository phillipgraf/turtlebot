import time

import rclpy  # ROS client library
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from utils.tb3_motion import *


# states
# 0: normal
# 1: on the front wall
class Tb3(Node):
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
        self.state = 0
        self.go = True
        self.front_search = True
        self.back_search = False
        self.right_search = False
        self.left_search = False
        self.counter = 0
        self.ang_vel_percent = 0
        self.lin_vel_percent = 0

    def vel(self, lin_vel_percent, ang_vel_percent=0):
        """ publishes linear and angular velocities in percent
        """
        # for TB3 Waffle
        MAX_LIN_VEL = 0.26  # m/s
        MAX_ANG_VEL = 1.82  # rad/s

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = MAX_LIN_VEL * lin_vel_percent / 100
        cmd_vel_msg.angular.z = MAX_ANG_VEL * ang_vel_percent / 100

        self.cmd_vel_pub.publish(cmd_vel_msg)
        self.ang_vel_percent = ang_vel_percent
        self.lin_vel_percent = lin_vel_percent

    def scan_callback(self, msg):
        """
        is run whenever a LaserScan msg is received
        """
        #### Degrees of laser view
        # 60 - 120 right side
        # 150 -210 behind
        # 240 - 300 left
        # -30 - 30 front

        print('state: ', self.state)
        print('\nmin Distance to front object:', min([msg.ranges[x] for x in range(-90, 90)]))
        print('min Distance to back object:', msg.ranges[180])
        print('Minimal distance front wall to back wall: ',
              min([msg.ranges[x] for x in range(-30, 30)]) + min([msg.ranges[x] for x in range(150, 210)]))
        print('Minimal distance left wall to right wall: ',
              min([msg.ranges[x] for x in range(-120, -60)]) + min([msg.ranges[x] for x in range(60, 120)]), '\n')

        min_dist_front = 0.25  # half robot
        min_dist_back = 0.17
        min_dist_left = 0.17
        min_dist_right = 0.16

        search_object(self, laser=msg.ranges, scan_range_front=min_dist_front, scan_range_back=min_dist_back,
                      scan_range_left=min_dist_left, scan_range_right=min_dist_right)

        if self.go:
            drive(self, 30)
            self.go = False

        if self.state == "object_front":
            self.counter += 1
            stop(self)
            self.right_search = True
            rotate(self, 10)

        if self.state == "object_right":
            drive(self, 15)
            self.front_search = True
            self.right_search = False

        if self.counter >= 2:
            stop(self)


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
