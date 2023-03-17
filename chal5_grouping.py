import os
from enum import Enum
import rclpy
from statistics import median
from rclpy.node import Node as Botnode
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from transforms3d.euler import quat2euler

# User defined functions
from utils.tb3_lds_laser import *
from utils.tb3_logs import *
from utils.tb3_motion import *
from utils.tb3_math import *
from utils.tb3_mapping import *
from utils.tb3_motion import *

class States(Enum):
    CHECK_FOR_GOAL = 0
    CHECK_FILTER_GROUPS = 1
    ROTATE_TO_PATH = 2.1
    ROTATE_TO_GOAL = 2.2
    DRIVE_UNTIL_BLOCKED = 3
    DRIVE_TO_GOAL = 4
    DO_NOTHING = 5

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

        self.ang_vel_percent = 0
        self.lin_vel_percent = 0

        # State management
        self.state = States.CHECK_FOR_GOAL

        # Saving tb3 sensor data
        self.pos = None
        self.orient = (0, 0, 0)
        self.beams = []
        self.beam_intensities = []

        # Variablees for path finding
        self.cell = [0, 0]
        self.cell_storage = []
        self.known_cells = []
        self.init_cell = True
        self.new_cell = True
        self.maze = None
        self.last_node = None
        self.node_id = str([1, 1])

        # Grouping variables
        self.groups = []
        self.filtered_groups = []

        # Rotation variables
        self.init_angle = 9999
        self.angle = 0
        self.rotation_tolerance = 0.5

        # Constants
        self.beam_distance = 1


    def odom_callback(self, msg):
        """
            is run whenever a Odometry msg is received
        """
        self.pos = msg.pose.pose.position
        self.orient = quat2euler([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                                  msg.pose.pose.orientation.w])
        if self.init_cell:
            self.cell[0] = 1
            self.cell[1] = 1
            self.cell_storage.append(self.cell[:])
            init_tree(self)
            self.init_cell = False
        else:
            current_cell = get_cell(self)
            if check_cell(self, get_cell(self)):
                self.known_cells = self.cell_storage[:]
                self.cell_storage.append(current_cell[:])
            path_creating(self)

        if self.state == States.CHECK_FOR_GOAL:
            stop(self)
            if detect_red_with_lds(self):
                self.state = States.ROTATE_TO_GOAL
            else:
                self.state = States.CHECK_FILTER_GROUPS
        elif self.state == States.CHECK_FILTER_GROUPS:
            calculate_groups(self, self.beam_distance)
            if len(self.groups) >= 1:
                self.filtered_groups = filter_groups(self, self.groups)
                if len(self.filtered_groups) >= 1:
                    if self.init_angle == 9999:
                        self.init_angle = self.orient[0]
                    self.angle = get_specific_degree_of_group(self.init_angle, self.filtered_groups[0])
                    self.state = States.ROTATE_TO_PATH
        elif self.state == States.ROTATE_TO_PATH:
            rotate(self, 15)
            if did_rotate_to_angle(self, self.angle, tolerance=self.rotation_tolerance):
                self.init_angle = 9999
                self.state = States.DRIVE_UNTIL_BLOCKED
        elif self.state == States.DRIVE_UNTIL_BLOCKED:
            drive(self, 30)
            if wall_in_range(self):
                self.state = States.CHECK_FOR_GOAL
        elif self.state == States.ROTATE_TO_GOAL:
            rotate(self, 15)
            if detect_red_with_lds_front(self):
                self.state = States.DRIVE_TO_GOAL
        elif self.state == States.DRIVE_TO_GOAL:
            drive(self, 30)
            if collide_with_wall(self): # TODO: this stops slightly before, in case of the timer
                self.state = States.DO_NOTHING
        elif self.state == States.DO_NOTHING:
            stop(self)

        diagnostics_final(self)

    def scan_callback(self, msg):
        """
            is run whenever a LaserScan msg is received
        """
        self.beams = msg.ranges
        self.beam_intensities = msg.intensities


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
