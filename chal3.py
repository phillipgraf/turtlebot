import rclpy  # ROS client library
from rclpy.node import Node as Botnode
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from utils.tb3_logs import diagnostics
from transforms3d.euler import quat2euler
import math

from utils.tb3_motion import drive, stop, rotate


class Tb3(Botnode):
    def __init__(self):
        super().__init__('tb3')

        self.pos = None
        self.orient = [-1, -1, -1]
        self.cmd_vel_pub = self.create_publisher(
            Twist,  # message type
            'cmd_vel',  # topic name
            1)  # history depth

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos_profile_sensor_data)

        self.pos = None
        self.orient = [-1, -1, -1]
        self.go = True
        self.front_search = True
        self.back_search = False
        self.right_search = False
        self.left_search = False
        self.ang_vel_percent = 0
        self.lin_vel_percent = 0
        self.angel_coefficient = 0.025
        self.state = -2

    def odom_callback(self, msg):
        self.pos = msg.pose.pose.position
        self.orient = quat2euler([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                                  msg.pose.pose.orientation.w])

        if self.go:
            drive(self, 20)
            self.go = False
        if self.pos.y > 0.80:
            stop(self)
            rotate(self, 10)
        if self.orient[0] > (math.radians(180) - self.angel_coefficient):
            stop(self)
            drive(self, 20)
        if self.pos.x < 0.15:
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
