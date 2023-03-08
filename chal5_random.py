import os
import rclpy
from statistics import median
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from utils.tb3_camera import detect_red
from utils.tb3_lds_laser import detect_red_with_lds, detect_red_with_lds_front, get_grouped, get_degree_of_random_group, get_red_beam
from utils.tb3_logs import diagnostics
from utils.tb3_motion import *
from transforms3d.euler import quat2euler
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError


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
        self.beams = []

        self.pos = None
        self.orient = [-1, -1, -1]
        self.front_beams = {}
        self.back_beams = {}
        self.left_beams = {}
        self.right_beams = {}
        self.op_beams = [(0, 0)]
        self.groups = []
        self.pre_rotate = 9999
        self.rot_goal = 9999
        self.once = True
        self.beam = (0, 0)
        self.beam_intensities = []

        self.image = None
        self.red_percentage = 0

        self.goal_road = False
        self.last_origin_degree = None

        self.drive_velocity = 20
        self.rotation_velocity = 5
        self.rotation_tolerance = 0.02
        self.rotation_clockwise = False

        self.beam_distance = 1
        self.max_beam_distance = 2

        self.front_distance = 0.45
        self.back_distance = 0.45
        self.right_distance = 0.45
        self.left_distance = 0.45

        self.state = -1
        """
            -1: Check the beams and create beam groups
            0: get beam for rotation
            1: rotate to selected beam
            2: drive until a wall
            3: Check for red walls
        """

    def img_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image = cv_image
            self.image_received = True
            detect_red(self)
        except CvBridgeError as e:
            print(e)
        self.image_received = False

    def odom_callback(self, msg):
        self.pos = msg.pose.pose.position
        self.orient = quat2euler([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                                  msg.pose.pose.orientation.w])

        if self.state == 0:
            # Check for highest beam
            self.beam = get_degree_of_random_group(self)
            self.state = 1
        elif self.state == 1:
            # Rotate the bot to the beam
            if self.beam is not None:
                self.rotation_clockwise = True if self.beam[0] >= 180 else False
                rotate_degree(self)
        elif self.state == 2:
            # Drive forward
            drive_until_wall(self)
        elif self.state == 3:
            if detect_red_with_lds(self):
                self.beam = get_red_beam(self)
                self.rotation_clockwise = True if self.beam[0] >= 180 else False
                self.state = 5
                stop(self)
            else:
                self.state = -1
                stop(self)
        elif self.state == 4:
            stop(self)
        elif self.state == 5:
            rotate_degree(self)
            if detect_red_with_lds_front(self):
                stop(self)
                self.state = 2
                self.goal_road = True
        diagnostics(self)

    def scan_callback(self, msg):
        """
            is run whenever a LaserScan msg is received
        """

        self.beams = msg.ranges
        self.beam_intensities = msg.intensities
        if self.state == -1:
            self.op_beams = [(x, self.beams[x]) for x in range(0, len(self.beams)) if
                             self.beams[x] > self.beam_distance]
            get_grouped(self)

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
