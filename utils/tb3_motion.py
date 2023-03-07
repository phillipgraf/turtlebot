import math

from utils.tb3_lds_laser import check_front_wall
from utils.tb3_math import rad, rad_overlap
from geometry_msgs.msg import Twist


def vel(tb3, lin_vel_percent, ang_vel_percent=0):
    """ publishes linear and angular velocities in percent
    """
    # for TB3 Waffle
    MAX_LIN_VEL = 0.26  # m/s
    MAX_ANG_VEL = 1.82  # rad/s

    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = MAX_LIN_VEL * lin_vel_percent / 100
    cmd_vel_msg.angular.z = MAX_ANG_VEL * ang_vel_percent / 100

    tb3.cmd_vel_pub.publish(cmd_vel_msg)
    tb3.ang_vel_percent = ang_vel_percent
    tb3.lin_vel_percent = lin_vel_percent

def rotate_dir(tb3):
    if tb3.rotation_clockwise:
        rotate(tb3, -tb3.rotation_velocity)
    else:
        rotate(tb3, tb3.rotation_velocity)

def in_tolerance(tb3):
    return tb3.rot_goal - tb3.rotation_tolerance <= tb3.orient[0] <= tb3.rot_goal + tb3.rotation_tolerance

def rotate_degree(tb3):
    if tb3.pre_rotate == 9999:
        tb3.pre_rotate = tb3.orient[0]
    if tb3.rotation_clockwise:
        rotate_dir(tb3)
        tb3.rot_goal = rad_overlap((tb3.pre_rotate * (180 / math.pi)) + tb3.beam[0])
        if in_tolerance(tb3):
            tb3.pre_rotate = 9999
            stop(tb3)
            tb3.state = 2
    else:
        rotate_dir(tb3)
        tb3.rot_goal = rad_overlap((tb3.pre_rotate * (180 / math.pi)) - tb3.beam[0])
        if in_tolerance(tb3):
            tb3.pre_rotate = 9999
            stop(tb3)
            tb3.state = 2

def drive_until_wall(tb3):
    drive(tb3, tb3.drive_velocity)
    if check_front_wall(tb3):
        stop(tb3)
        tb3.state = -1

def stop(tb3: object):
    """
    Stop the bot set velocity and rotation to 0.
    And stop scan search.

    Set state to "stop".
    """
    vel(tb3, 0, 0)
    tb3.rot = False


def start_search(tb3):
    """
    Start search in all directions
    :param tb3: bot object
    """
    tb3.front_search = True
    tb3.back_search = True
    tb3.right_search = True
    tb3.left_search = True


def drive(tb3: object, velocity: int):
    """
    Drive the bot with given velocity.
    :param tb3: The bot which should drive
    :param velocity: integer Velocity is given in percentages. Positiv Integer drive forward, negative integer drive backward

    Set state to "drive".
    """
    if velocity == 0:
        stop(tb3)
        return
    vel(tb3, velocity, 0)


def rotate(tb3: object, rotation_velocity: int):
    """
    Rotate the bot with the given rotation speed.
    :param tb3: The bot.
    :param rotation_velocity: The velocity of the rotation

    Set state to "rotate".
    """
    if rotation_velocity == 0:
        stop(tb3)
        return
    vel(tb3, 0, rotation_velocity)


def get_and_set_view(tb3: object, orient, angel_coefficient=0.5):
    """
    Get the view of the bot.

    #orient_north = 90
    #orient_west = 180
    #orient_east = 0
    #orient_south = -90

    And set VIEW to the value
    :param tb3:
    """

    if rad(90 - angel_coefficient) < orient[0] < rad(90 + angel_coefficient):
        tb3.VIEW = "north"
    elif rad(180 - angel_coefficient) < orient[0] < rad(180 + angel_coefficient):
        tb3.VIEW = "west"
    elif rad(0 - angel_coefficient) < orient[0] < rad(0 + angel_coefficient):
        tb3.VIEW = "east"
    elif rad(-90 - angel_coefficient) < orient[0] < rad(-90 + angel_coefficient):
        tb3.VIEW = "south"


def rotate_90_degree(tb3, direction_to_move, orient_of_bot):
    """
    Rotate to the bot to the given point of the compass. Currently only "north", "west", "south" and "east" implemented.
    Also get_and_set the view of the bot.
    
    Points of the compass:
    - "north"
    - "west"
    - "south"
    - "east"
    
    :param tb3: Bot object.
    :param direction_to_move: Positiv integer for left
                              Negativ integer for right
    :param orient_of_bot:  Current orientation state of the bot
    """
    if tb3.VIEW == "north":
        rotate(tb3, direction_to_move)
        get_and_set_view(tb3, orient_of_bot)
        # direction positiv rotate to the left
        if direction_to_move > 0:
            if tb3.VIEW == "west":
                stop(tb3)
                start_search(tb3)
        # direction negative rotate to right
        elif direction_to_move < 0:
            if tb3.VIEW == "east":
                stop(tb3)
                start_search(tb3)
        else:
            return

    elif tb3.VIEW == "west":
        rotate(tb3, direction_to_move)
        get_and_set_view(tb3, orient_of_bot)
        if direction_to_move > 0:
            # if min(orient[0], -orient[0]) > rad(tb3.orient_south):
            if tb3.VIEW == "south":
                stop(tb3)
                start_search(tb3)
        elif direction_to_move < 0:
            if tb3.VIEW == "north":
                stop(tb3)
                start_search(tb3)
        else:
            return

    elif tb3.VIEW == "south":
        rotate(tb3, direction_to_move)
        get_and_set_view(tb3, orient_of_bot)
        if direction_to_move > 0:
            if tb3.VIEW == "east":
                stop(tb3)
                start_search(tb3)
        elif direction_to_move < 0:
            if tb3.VIEW == "west":
                stop(tb3)
                start_search(tb3)
        else:
            return

    elif tb3.VIEW == "east":
        rotate(tb3, direction_to_move)
        get_and_set_view(tb3, orient_of_bot)
        if direction_to_move > 0:
            if tb3.VIEW == "north":
                stop(tb3)
                start_search(tb3)
        elif direction_to_move < 0:
            if tb3.VIEW == "south":
                stop(tb3)
                start_search(tb3)
        else:
            return
