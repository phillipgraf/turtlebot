import time

import rclpy  # ROS client library
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from utils.tb3_motion import *
from transforms3d.euler import quat2euler
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


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

        # self.odom_sub = self.create_subscription(
        #     Odometry,
        #     'odom',
        #     self.odom_callback,
        #     qos_profile_sensor_data)

        self.image = None
        self.bridge = CvBridge()
        self.image_received = False
        self.img_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.img_callback,
            qos_profile_sensor_data)

        # allows packet loss
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

    def img_callback(self, msg):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.image_received = True
        self.image = cv_image

        detect_red(self)
        start_video(self)
        rotate(self, 5000)


    def odom_callback(self, msg):
        #print("State", msg)
        pos = msg.pose.pose.position
        orient = quat2euler([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        print("Postion", pos)
        print("Orientation", orient)
        if self.go:
            drive(self, 20)
            self.go = False
        if pos.y > 0.80:
            stop(self)
            rotate(self, 20)
        if orient[0] > 3.1:
            stop(self)
            drive(self, 20)
        if pos.x < 0.15:
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

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
