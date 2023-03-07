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
import math


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

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos_profile_sensor_data)

        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,  # function to run upon message arrival
            qos_profile_sensor_data)  # allows packet loss

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
        self.rot = False
        self.front_search = True
        self.back_search = True
        self.right_search = True
        self.left_search = True
        self.object_front = False
        self.object_back = False
        self.object_left = False
        self.object_right = False
        self.counter = 0
        self.ang_vel_percent = 0
        self.lin_vel_percent = 0
        self.image = None
        self.VIEW = "north"
        self.color = ""
        self.rotate_direction = None
        self.pos = None
        self.orient = [-1, -1, -1]
        self.groups = None
        self.new_group_1 = []
        self.new_group_2 = []
        self.pos = None
        self.dead_end = False

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

        start_video(self)
        if not self.rot:
            detect_red(self)


    def odom_callback(self, msg):
        self.pos = msg.pose.pose.position
        orient = quat2euler([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                             msg.pose.pose.orientation.w])

        # print("Postion", pos)
        print("Robot view:", self.VIEW)


        if self.go:
            get_and_set_view(self, orient)
            drive(self, 30)
            start_search(self)
            self.go = False

        if self.rot:
            rotate_90_degree(self, self.rotate_direction, orient)
        else:
            if self.color == "red":
                if self.object_front:
                    stop(self)
                else:
                    drive(self, 20)
            else:
                if self.object_front and self.object_left:
                    stop(self)
                    self.rot = True
                    self.rotate_direction = -10
                elif self.object_front and self.object_right:
                    stop(self)
                    self.rot = True
                    self.rotate_direction = 10
                elif self.object_front:
                    stop(self)
                    self.rot = True
                    self.rotate_direction = 20
                elif self.object_right:
                    stop(self)
                    drive(self, 20)
                elif self.object_left:
                    stop(self)
                    drive(self, 20)
                else:
                    self.go = True

    def scan_callback(self, msg):
        """
        is run whenever a LaserScan msg is received
        """
        #### Degrees of laser view
        # 60 - 120 right side
        # 150 -210 behind
        # 240 - 300 left
        # -30 - 30 front

        min_dist_front = 0.32
        min_dist_back = 0.32
        min_dist_left = 0.32
        min_dist_right = 0.32

        search_object(self, laser=msg.ranges, scan_range_front=min_dist_front, scan_range_back=min_dist_back,
                      scan_range_left=min_dist_left, scan_range_right=min_dist_right)

        get_grouped_beams(self, msg.ranges)
        print("SEARCH DEAD-END")
        check_dead_end(self, self.groups[0], msg)

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
