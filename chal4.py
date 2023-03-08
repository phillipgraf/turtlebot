import time

import rclpy  # ROS client library
from rclpy.node import Node as Botnode
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from utils.tb3_camera import start_video, detect_red
from utils.tb3_lds_laser import search_object, get_grouped_beams, check_dead_end
from utils.tb3_motion import *
from utils.tb3_logs import diagnostics
from utils.tb3_mapping import *
from transforms3d.euler import quat2euler
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError


class Tb3(Botnode):
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

        self.state = -4
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
        self.new_group_1 = []
        self.new_group_2 = []
        self.pos = None
        self.dead_end = False

        self.min_dist_front = 0.5
        self.min_dist_back = 0.5
        self.min_dist_left = 1
        self.min_dist_right = 1
        self.max_dist_front = 0
        self.max_dist_left = 0
        self.max_dist_back = 0
        self.max_dist_right = 0

        self.beam_distance = 1

        self.first_time_center = False
        self.cell = [0, 0]
        self.cell_storage = []
        self.cell_count = 0
        self.known_cells = []
        self.init_cell = True
        self.groups = []
        self.beams = []


    def img_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        self.image_received = True
        self.image = cv_image

        #start_video(self)
        if not self.rot:
            detect_red(self)

    def odom_callback(self, msg):
        self.pos = msg.pose.pose.position
        orient = quat2euler([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                             msg.pose.pose.orientation.w])

        if self.init_cell:
            # init start cell
            self.cell[0] = 1
            self.cell[1] = 1
            self.cell_storage.append(self.cell[:])
            self.init_cell = False
        else:
            current_cell = get_cell(self)
            if check_cell(self, get_cell(self)):
                self.known_cells = self.cell_storage[:]
                self.cell_storage.append(current_cell[:])

        if self.go:
            self.first_time_center = False
            get_and_set_view(self, orient, angel_coefficient=2)
            drive(self, 20)
            start_search(self)
            self.go = False

        # get_grouped_beams(self, self.beams)
        # dead_ends = [group for group in self.groups if check_dead_end(self, group, visualize=True)]
        # if len(dead_ends) >= 1:
        #     print(f"DEADENDS:\n\n{dead_ends}")
        #     for end in dead_ends:
        #         if 0 in end:
        #             self.object_front = True
        #         elif 90 in end:
        #             self.object_right = True
        #         elif 180 in end:
        #             self.object_back = True
        #         elif 270 in end:
        #             self.object_left = True
        if self.rot:
            rotate_90_degree(self, self.rotate_direction, orient)
        else:
            if self.color == "red":
                if self.object_front:
                    stop(self)
                else:
                    drive(self, 30)
            else:
                if not cell_center(self, thresh=0.2):
                    drive(self, 30)
                    self.first_time_center = True
                elif self.first_time_center:
                    self.first_time_center = False
                    if self.object_front and self.object_left:
                        stop(self)
                        self.rot = True
                        self.rotate_direction = -5
                    elif self.object_front and self.object_right:
                        stop(self)
                        self.rot = True
                        self.rotate_direction = 5
                    # elif self.object_front:
                        #stop(self)
                    #     self.rot = True
                    #     self.rotate_direction = 15
                    elif self.object_left and self.object_right:
                        drive(self, 30)
                    elif not self.object_right:
                        stop(self)
                        self.rot = True
                        self.rotate_direction = -5
                    elif not self.object_left:
                        stop(self)
                        self.rot = True
                        self.rotate_direction = 5
                    # elif self.object_right:
                    #     stop(self)
                    #     drive(self, 30)
                    # elif self.object_left:
                    #     stop(self)
                    #     drive(self, 30)
                    else:
                        self.go = True
            diagnostics(self)

    def scan_callback(self, msg):
        """
        is run whenever a LaserScan msg is received
        """
        # Degrees of laser view
        # 60 - 120 right side
        # 150 -210 behind
        # 240 - 300 left
        # -30 - 30 front
        self.beams = msg.ranges

        search_object(self, laser=msg.ranges)


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
