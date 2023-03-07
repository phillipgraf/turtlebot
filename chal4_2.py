import rclpy  # ROS client library
from statistics import median
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import sys
import os
import time
from itertools import islice

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

                self.state = 0
                self.ang_vel_percent = 0
                self.lin_vel_percent = 0
                self.image = None
                self.color = ""

                self.pos = None
                self.orient = [-1, -1, -1]
                self.front_beams = {}
                self.back_beams = {}
                self.left_beams = {}
                self.right_beams = {}
                self.op_beams = [(0,0)]
                self.groups = []
                self.pre_rotate = 9999
                self.rot_goal = 9999
                self.once = True
                self.beam = (0,0)
                self.beam_intensities = []

                self.last_origin_degree = None

                self.drive_velovity = 20
                self.rotation_velocity = 5
                self.rotation_tolerance = 0.02
                self.rotation_clockwise = False

                self.beam_distance = 1.4

                self.front_distance = 0.45

                self.state = -1
                """
                -1: Check the beams and create beam groups
                0: get beam for rotation
                1: rotate to selected beam
                2: drive until a wall
                3: Check for red walls
                """

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
                        self.image = cv_image
                        self.image_received = True
                        detect_red(self)
                except CvBridgeError as e:
                        print(e)
                self.image_received = False

        def stop(self):
                self.vel(0, 0)

        def rotate(self):
                if self.rotation_clockwise:
                        self.vel(0, -self.rotation_velocity)
                else:
                        self.vel(0, self.rotation_velocity)

        def odom_callback(self, msg):
                self.pos = msg.pose.pose.position
                self.orient = quat2euler([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])

                if self.state == 0:
                        # Check for highest beam
                        self.beam = self.get_degree_of_prefered_group()
                        self.state = 1
                elif self.state == 1:
                        # Rotate the bot to the beam
                        if self.beam != None:
                                self.rotation_clockwise = True if self.beam[0] >= 180 else False
                                self.rotate_degree()
                elif self.state == 2:
                        # Drive forward
                        self.drive_until_wall()
                elif self.state == 3:
                        self.rotate()
                        if detect_red(self, fill_percentage=0.01, red_in_center=True):
                                self.stop()
                elif self.state == 4:
                        self.stop()
                self.diagnostics()

        def drive_until_wall(self):
                self.drive()
                if self.check_front_wall():
                        self.stop()
                        self.state = -1

        def check_front_wall(self):
                return any(self.beams[x] < self.front_distance for x in range(-30, 30))

        def check_back_wall(self):
                return any(self.beams[x] < self.front_distance for x in range(150, 210))

        def check_left_wall(self):
                return any(self.beams[x] < self.front_distance for x in range(240, 300))

        def check_right_wall(self):
                return any(self.beams[x] < self.front_distance for x in range(60, 120))

        def drive(self):
                self.vel(self.drive_velovity, 0)

        def rotate_degree(self):
                if self.pre_rotate == 9999:
                        self.pre_rotate = self.orient[0]
                if self.rotation_clockwise:
                        self.rotate()
                        self.rot_goal = rad((self.pre_rotate*(180/math.pi)) + self.beam[0])
                        if self.in_tolerance():
                                self.pre_rotate = 9999
                                self.stop()
                                self.state = 2
                else:
                        self.rotate()
                        self.rot_goal = rad((self.pre_rotate*(180/math.pi)) - self.beam[0])
                        if self.in_tolerance():
                                self.pre_rotate = 9999
                                self.stop()
                                self.state = 2

        def in_tolerance(self):
                return self.rot_goal - self.rotation_tolerance <= self.orient[0] <= self.rot_goal + self.rotation_tolerance

        def scan_callback(self, msg):
                """
                is run whenever a LaserScan msg is received
                """
                # min_dist_front = 0.32
                # min_dist_back = 0.32
                # min_dist_left = 0.32
                # min_dist_right = 0.32

                self.beams = msg.ranges
                self.beam_intensities = msg.intensities
                if self.state == -1:
                        self.op_beams = [(x, self.beams[x]) for x in range(0, len(self.beams)) if self.beams[x] > self.beam_distance]
                        self.get_grouped_beams()
                self.diagnostics()

        def get_grouped_beams(self):
                if len(self.op_beams) == 0:
                        return
                self.groups = [[self.op_beams[0][0]]]
                idx = self.op_beams[0][0]
                for x in self.op_beams[1:]:
                        id = x[0]
                        ab = abs(idx - id)
                        if ab == 1:
                                idx = id
                                self.groups[-1].append(id)
                        else:
                                idx = id
                                self.groups.append([id])
                if len(self.groups) >= 2 and self.groups[0][0] == 0 and self.groups[-1][-1] == 359:
                        self.groups = [self.groups[-1] + self.groups[0]] + self.groups[1:-1]
                if len(self.groups) >= 1:
                        self.state = 0

        def get_degree_of_prefered_group(self):
                saved_g = []
                highest = 0
                for g in self.groups:
                        for x in g:
                                if self.beams[x] > highest:
                                        saved_g = g
                                        highest = self.beams[x]
                med = int(median(saved_g))
                if self.last_origin_degree == None:
                        return (med, self.beams[med])
                if self.last_origin_degree - 15 < med < self.last_origin_degree + 15:
                        # Forbidden direction, go to next group
                        if len(self.groups) == 1:
                                self.stop()
                                self.state = 4
                                return

        def diagnostics(self):
                if self.pos != None and self.orient != None:
                        try:
                                os.system("clear")
                                print(f"{get_title()}")
                                print(f"X{' ' * len(str(self.pos.x))}\tY{' ' * len(str(self.pos.y))}\tZ{' ' * len(str(self.pos.z))}\t|\tRotX{' ' * len(str(self.orient[0]))}\tRotY{' ' * (len(str(self.orient[1]))-3)}\tRotZ")
                                print(f"{self.pos.x}\t{self.pos.y}\t{self.pos.z}\t|\t{self.orient[0]}\t{self.orient[1]}\t{self.orient[2]}")
                                print(f"{get_state(self.state)}")
                                if self.state == 0:
                                        print(f"List of beam groups that extend the distance: {self.beam_distance}")
                                        print(f"#Beams\t|\t <\t>")
                                        print(f"{'-' * 35}")
                                        for g in self.groups:
                                                print(f"{len(g)}\t|\t{g[0]}\t{g[-1]}")
                                if self.state == 1:
                                        print(f"Rotation speed: {self.rotation_velocity}")
                                        print(f"Rotation tolerance: {self.rotation_tolerance}")
                                        print(f"pre rotation value: {self.pre_rotate * (180/math.pi)}")
                                        print(f"rotation goal: {self.rot_goal} || {self.rot_goal * (180/math.pi)}")
                                        print(f"Target Beam: {self.beam[0]} >> {self.beam[1]}")
                                        print(f"latest Origin: {self.last_origin_degree}")
                                if self.state == 2:
                                        print(f"Speed: {self.drive_velovity}")
                                        print(f"Collisions:\n")
                                        print(f"\t{self.check_front_wall()}")
                                        print(f"\t  ^")
                                        print(f"{self.check_left_wall()} <\t\t> {self.check_right_wall()}")
                                        print(f"\t  v")
                                        print(f"\t{self.check_back_wall()}")
                        except Exception as e:
                                print(e)

def rad(deg):
        if deg > 180:
                deg -= 360
        if deg < -180:
                deg += 360
        return math.radians(deg)

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
