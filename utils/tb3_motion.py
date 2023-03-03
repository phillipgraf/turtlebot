import math

import numpy as np
import cv2
import os


def stop(tb3: object):
    """
    Stop the bot set velocity and rotation to 0.
    And stop scan search.

    Set state to "stop".
    """
    tb3.vel(0, 0)
    tb3.rot = False
    print("Bot stopped")
    tb3.state = "stop"


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
    tb3.vel(velocity, 0)
    tb3.state = "drive"
    print("Bot driving forward" if velocity > 0 else "Bot driving backwards")


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
    tb3.vel(0, rotation_velocity)
    tb3.state = "rotate"
    print("Bot is rotating to the left" if rotation_velocity > 0 else "Bot is rotating to the right side")


def rad(deg):
    """
    Convert degree to radians
    :param deg: degree value
    :return: converted radian
    """
    return math.radians(deg)


def search_object(tb3: object, laser, scan_range_front=0.0, scan_range_back=0.0, scan_range_left=0.0,
                  scan_range_right=0.0):
    """
    Search for a object with the given laser in the specifiy scan_range.

    :param scan_range_right:int Distance to detection at the right side.
    :param scan_range_left:int Distance to detection at the left side.
    :param scan_range_front:int Distance to detection at the front side.
    :param scan_range_back:int Distance to detection at the back side.
    :param tb3: Bot object.
    :param laser: Array of all laser data.

    # 60 - 120 right side
    # 150 -210 behind
    # 240 - 300 left
    # -30 - 30 front

    Set state to "object".
    """

    if tb3.front_search:
        if scan_range_front >= laser[0]:
            tb3.object_front = True
            object_position = "front"
            print("Attention! Object at {}.".format(object_position))
        else:
            tb3.object_front = False
            print("No Object in front.")

    if tb3.back_search:
        if scan_range_back >= laser[180]:
            tb3.object_back = True
            object_position = "back"
            print("Attention! Object at {}.".format(object_position))
        else:
            tb3.object_back = False
            print("No Object in behind.")

    if tb3.right_search:
        if scan_range_right >= laser[-90]:
            tb3.object_right = True
            object_position = "right"
            print("Attention! Object at {}.".format(object_position))
        else:
            tb3.object_right = False
            print("No Object in right.")

    if tb3.left_search:
        if scan_range_left >= laser[90]:
            tb3.object_left = True
            object_position = "left"
            print("Attention! Object at {}.".format(object_position))
        else:
            tb3.object_left = False
            print("No Object in left.")


def detect_red(tb3: object):
    """
    Detect the color red of an object in the front between the rgb ranges:
    lower: 0, 0, 120
    upper: 10, 10, 130

    Set color to "red".  If no red empty string "".

    :param tb3: Bot object
    """
    lower_red = np.array([0, 0, 120], np.uint8)
    upper_red = np.array([10, 10, 130], np.uint8)
    mask = cv2.inRange(tb3.image, lower_red, upper_red)
    if (mask == 255).sum() > 1:
        tb3.color = "red"
        print("Color detected: RED")
    else:
        print("Color detected: NONE")
        tb3.color = ""


def start_video(tb3: object):
    """
    Show a video of the camera view.
    :param tb3: Bot object.
    """
    cv2.imshow("Video", tb3.image)
    if cv2.waitKey(10) & 0xFF == ord('q'):
        cv2.destroyAllWindows()


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
    # min(orient[0], -orient[0])

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
            print("Move to west")
            if tb3.VIEW == "west":
                stop(tb3)
                start_search(tb3)
        # direction negative rotate to right
        elif direction_to_move < 0:
            print("Move to east")
            if tb3.VIEW == "east":
                stop(tb3)
                start_search(tb3)
        else:
            return

    elif tb3.VIEW == "west":
        rotate(tb3, direction_to_move)
        get_and_set_view(tb3, orient_of_bot)
        if direction_to_move > 0:
            print("Move to south")
            # if min(orient[0], -orient[0]) > rad(tb3.orient_south):
            if tb3.VIEW == "south":
                stop(tb3)
                start_search(tb3)
        elif direction_to_move < 0:
            print("Move to north")
            if tb3.VIEW == "north":
                stop(tb3)
                start_search(tb3)
        else:
            return

    elif tb3.VIEW == "south":
        rotate(tb3, direction_to_move)
        get_and_set_view(tb3, orient_of_bot)
        if direction_to_move > 0:
            print("Move to east")
            if tb3.VIEW == "east":
                stop(tb3)
                start_search(tb3)
        elif direction_to_move < 0:
            print("Move to west")
            if tb3.VIEW == "west":
                stop(tb3)
                start_search(tb3)
        else:
            return

    elif tb3.VIEW == "east":
        rotate(tb3, direction_to_move)
        get_and_set_view(tb3, orient_of_bot)
        if direction_to_move > 0:
            print("Move to north")
            if tb3.VIEW == "north":
                stop(tb3)
                start_search(tb3)
        elif direction_to_move < 0:
            print("Move to south")
            if tb3.VIEW == "south":
                stop(tb3)
                start_search(tb3)
        else:
            return


def get_grouped_beams(tb3: object, beams):
    """

    :param tb3:
    :param op_beams:
    :return:
    """
    op_beams = [(x, beams[x]) for x in range(0, len(beams)) if beams[x] > 1]
    if len(op_beams) == 0:
        return
    tb3.groups = [[op_beams[0][0]]]
    idx = op_beams[0][0]
    for x in op_beams[1:]:
        id = x[0]
        ab = abs(idx - id)
        if ab == 1:
            idx = id
            tb3.groups[-1].append(id)
        else:
            idx = id
            tb3.groups.append([id])
    if len(tb3.groups) >= 2 and tb3.groups[0][0] == 0 and tb3.groups[-1][-1] == 359:
        tb3.groups = [tb3.groups[-1] + tb3.groups[0]] + tb3.groups[1:-1]


def diagnostics(tb3):
    if tb3.pos is not None and tb3.orient is not None:
        try:
            os.system("clear")
            print(
                f"X{' ' * len(str(tb3.pos.x))}\tY{' ' * len(str(tb3.pos.y))}\tZ{' ' * len(str(tb3.pos.z))}\t|\tRotX{' ' * len(str(tb3.orient[0]))}\tRotY{' ' * (len(str(tb3.orient[1])) - 3)}\tRotZ")
            print(f"{tb3.pos.x}\t{tb3.pos.y}\t{tb3.pos.z}\t|\t{tb3.orient[0]}\t{tb3.orient[1]}\t{tb3.orient[2]}")
            print("\n")
            print(f"List of beam groups that extend the range of 1m")
            print(f"#Beams\t|\t <\t>")
            print(f"{'-' * 35}")
            for g in tb3.groups:
                print(f"{len(g)}\t|\t{g[0]}\t{g[-1]}")
            print(f"Rotation Logs (9999 is the default):\n\n")
            print(f"\nRadian calculations")
            print(f"Degree\tRadian")
            rads = [0, 90, 180, 60, 195, 270, 359]
            for r in rads:
                print(f"{r}\t{rad(r)}")
            print(f"pre rotation value: {tb3.pre_rotate}")
            print(f"rotation goal: {tb3.rot_goal}")
        except:
            pass


def get_laser_endpoint(start_x, start_y, len_laser):
    """
    Get the postion of the endpoint of the laser
    :param tb3: Bot object.
    """
    x = start_x + len_laser * math.cos(0)
    y = start_y + len_laser * math.sin(0)
    point = [0] * 2
    point[0] = x
    point[1] = y
    print("Laser endpoint", point)
    return point


def compute_length(point_1, point_2):
    """
    Compute the length between 2 points

    Important for the detection of an dead end
    :param point_1: dict(x,y)
    :param point_2: dict(x,y)
    :return: the length from point1 to point2
    """
    x1 = point_1[0]
    x2 = point_2[0]
    y1 = point_1[1]
    y2 = point_2[1]
    print(point_1)
    print(point_2)
    abstand = math.sqrt((x2 - x1) + (y2 - y1))
    return abstand


def check_equilateral(a, b, c):
    """
    Check if 3 lengths have the same length.

    Important for the detection of a dead end.
    :param a: int length
    :param b:int length
    :param c: int length
    :return: lengths
    """
    return a == b and b == c


def compute_equilateral(point1, point2, point3):
    """
    Check if the 3 points create triangle with same sites
    :param tb3: Bot object.
    :param point1: dict(x,y)
    :param point2: dict(x,y)
    :param point3: dict(x,y)
    :return: True if triangle detected, False if not
    """
    a = compute_length(point1, point2)
    b = compute_length(point2, point3)
    c = compute_length(point3, point1)

    if check_equilateral(a, b, c):
        return True
    else:
        return False

def check_dead_end(tb3, beam_group, msg):
    end_points = []
    for beam in beam_group:
        end_points.append(get_laser_endpoint(tb3.pos.x, tb3.pos.y, msg.ranges[beam]))

    for point1 in end_points:
        for point2 in end_points:
            for point3 in end_points:
                if point3 != point1 and point3 != point2 and point2 != point1:
                    if compute_equilateral(point1, point2, point3):
                        tb3.dead_end = True
                    else:
                        tb3.dead_end = False
