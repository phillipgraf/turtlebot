import math

import numpy as np
import cv2
import os
import matplotlib.pyplot as plt


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


def detect_red(tb3: object, fill_percentage = 0.1, red_in_center = False):
    """
    Detect the color red of an object in the front between the rgb ranges:
    lower: 0, 0, 120
    upper: 10, 10, 130

    Set color to "red".  If no red empty string "".

    :param tb3: Bot object
    """
    lower_red = np.array([0, 0, 100], np.uint8)
    upper_red = np.array([10, 10, 180], np.uint8)
    mask = cv2.inRange(tb3.image, lower_red, upper_red)
    print(mask)
    print(f"#number: ")
    tb3.colorsum = (mask == 255).sum()
    needed_pxl = tb3.image.shape[0] * tb3.image.shape[1] * fill_percentage
    tb3.needed_pxl = needed_pxl
    if (mask == 255).sum() > needed_pxl:
        tb3.img_shape = mask.shape
        tb3.color = "red"
        if red_in_center:
            if (mask.T[:5] != 255).sum() == 0 and (mask.T[-5:] != 255).sum() == 0:
                return True
            else:
                return False
        return True
        # print("Color detected: RED")
    else:
        return False
        # print("Color detected: NONE")
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

def get_title():
    return """
        ,----,
      ,/   .`|                                                                                          .--,-``-.
    ,`   .'  :                        ___     ,--,                                     ___             /   /     '.
  ;    ;     /                      ,--.'|_ ,--.'|               ,---,               ,--.'|_          / ../        ;
.'___,/    ,'        ,--,  __  ,-.  |  | :,'|  | :             ,---.'|      ,---.    |  | :,'         \ ``\  .`-    '
|    :     |       ,'_ /|,' ,'/ /|  :  : ' ::  : '             |   | :     '   ,'\   :  : ' :          \___\/   \   :
;    |.';  ;  .--. |  | :'  | |' |.;__,'  / |  ' |      ,---.  :   : :    /   /   |.;__,'  /                \   :   |
`----'  |  |,'_ /| :  . ||  |   ,'|  |   |  '  | |     /     \ :     |,-..   ; ,. :|  |   |                 /  /   /
    '   :  ;|  ' | |  . .'  :  /  :__,'| :  |  | :    /    /  ||   : '  |'   | |: ::__,'| :                 \  \   \
    |   |  '|  | ' |  | ||  | '     '  : |__'  : |__ .    ' / ||   |  / :'   | .; :  '  : |__           ___ /   :   |
    '   :  |:  | : ;  ; |;  : |     |  | '.'|  | '.'|'   ;   /|'   : |: ||   :    |  |  | '.'|         /   /\   /   :
    ;   |.' '  :  `--'   \  , ;     ;  :    ;  :    ;'   |  / ||   | '/ : \   \  /   ;  :    ;        / ,,/  ',-    .
    '---'   :  ,      .-./---'      |  ,   /|  ,   / |   :    ||   :    |  `----'    |  ,   /         \ ''\        ;
             `--`----'               ---`-'  ---`-'   \   \  / /    \  /              ---`-'           \   \     .'
                                                       `----'  `-'----'                                 `--`-,,-'
"""

def get_state(state):
    # https://patorjk.com/software/taag/#p=display&f=Small&t=State%3A%206
    # Font "Small"
    if state == -1:
        return """
  ___ _        _       _         _    ___                   _             _
 / __| |_ __ _| |_ ___(_)  ___  / |  / __|_ _ ___ _  _ _ __(_)_ _  __ _  | |__  ___ __ _ _ __  ___
 \__ \  _/ _` |  _/ -_)_  |___| | | | (_ | '_/ _ \ || | '_ \ | ' \/ _` | | '_ \/ -_) _` | '  \(_-<
 |___/\__\__,_|\__\___(_)       |_|  \___|_| \___/\_,_| .__/_|_||_\__, | |_.__/\___\__,_|_|_|_/__/
                                                      |_|         |___/
    """
    elif state == 0:
        return """
  ___ _        _       _    __    ___      _        _     _                         _
 / __| |_ __ _| |_ ___(_)  /  \  / __| ___| |___ __| |_  | |__  ___ __ _ _ __    __| |___ __ _ _ _ ___ ___
 \__ \  _/ _` |  _/ -_)_  | () | \__ \/ -_) / -_) _|  _| | '_ \/ -_) _` | '  \  / _` / -_) _` | '_/ -_) -_)
 |___/\__\__,_|\__\___(_)  \__/  |___/\___|_\___\__|\__| |_.__/\___\__,_|_|_|_| \__,_\___\__, |_| \___\___|
                                                                                         |___/
    """
    elif state == 1:
        return """
  ___ _        _       _   _   ___     _        _         _            _
 / __| |_ __ _| |_ ___(_) / | | _ \___| |_ __ _| |_ ___  | |_ ___   __| |___ __ _ _ _ ___ ___
 \__ \  _/ _` |  _/ -_)_  | | |   / _ \  _/ _` |  _/ -_) |  _/ _ \ / _` / -_) _` | '_/ -_) -_)
 |___/\__\__,_|\__\___(_) |_| |_|_\___/\__\__,_|\__\___|  \__\___/ \__,_\___\__, |_| \___\___|
                                                                            |___/
    """
    elif state == 2:
        return """
  ___ _        _       _   ___   ___      _
 / __| |_ __ _| |_ ___(_) |_  ) |   \ _ _(_)_ _____
 \__ \  _/ _` |  _/ -_)_   / /  | |) | '_| \ V / -_)
 |___/\__\__,_|\__\___(_) /___| |___/|_| |_|\_/\___|

    """
    elif state == 3:
        return """
  ___ _        _       _   ____   ___ _           _   _              __                          _
 / __| |_ __ _| |_ ___(_) |__ /  / __| |_  ___ __| |_(_)_ _  __ _   / _|___ _ _   __ _ ___  __ _| |
 \__ \  _/ _` |  _/ -_)_   |_ \ | (__| ' \/ -_) _| / / | ' \/ _` | |  _/ _ \ '_| / _` / _ \/ _` | |
 |___/\__\__,_|\__\___(_) |___/  \___|_||_\___\__|_\_\_|_||_\__, | |_| \___/_|   \__, \___/\__,_|_|
                                                            |___/                |___/
    """
    elif state == 4:
        return """
  ___ _        _       _   _ _    ___      _        _                         _
 / __| |_ __ _| |_ ___(_) | | |  | _ ) ___| |_   __| |_ ___ _ __ _ __  ___ __| |
 \__ \  _/ _` |  _/ -_)_  |_  _| | _ \/ _ \  _| (_-<  _/ _ \ '_ \ '_ \/ -_) _` |
 |___/\__\__,_|\__\___(_)   |_|  |___/\___/\__| /__/\__\___/ .__/ .__/\___\__,_|
                                                           |_|  |_|
    """
    elif state == 5:
        return """
  ___ _        _       _   ___
 / __| |_ __ _| |_ ___(_) | __|
 \__ \  _/ _` |  _/ -_)_  |__ \
 |___/\__\__,_|\__\___(_) |___/
    """
    elif state == 6:
        return """
  ___ _        _       _    __
 / __| |_ __ _| |_ ___(_)  / /
 \__ \  _/ _` |  _/ -_)_  / _ \
 |___/\__\__,_|\__\___(_) \___/
    """
    else:
        return ""

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


def get_laser_endpoint(start_x, start_y, len_laser, angular):
    """
    Get the postion of the endpoint of the laser
    :param tb3: Bot object.
    """
    x = start_x + len_laser * math.cos(rad(angular))
    y = start_y + len_laser * math.sin(rad(angular))

    point = [0] * 2
    point[0] = x
    point[1] = y
    print("Laser endpoint", point)
    return point


def check_dead_end(tb3, beam_group, msg, find_points_threshold=0.01, wall_threshold=5, visualize=False):
    """
    Set status tb3.dead_end to True or False, if the Bot found a dead end in this specific Group set the status
    tb3.dead_end to True

    :param tb3: Bot Object.
    :param beam_group:dict The specific Group.
    :param msg:dict Message you get from the turtlebot
    :param find_points_threshold:int Threshold to find points of a wall. Default: 0.01
    :param wall_threshold: int Threshold, how much points are needed to delcare a point_cloud as a wall. Default 5
    :param visualize:boolean Default False. Set to True if you want a visucalization of the laserpoints
    :return:
    """
    end_points = []
    x_axis = []
    y_axis = []
    same_x_1 = []
    same_x_2 = []
    same_y_1 = []
    same_y_2 = []
    i = 0
    wall = 0

    # get endpoints of the lasers in the laser group
    for beam in beam_group:
        end_point = get_laser_endpoint(tb3.pos.x, tb3.pos.y, msg.ranges[beam], beam)
        x_axis.append(end_point[0])
        y_axis.append(end_point[1])
        end_points.append(end_point)
        i += 1
        print(i)

    # split the endpoints in 2 Groups
    # y-Groups, points which have the same y-axis
    # x-group, points which have the same x-axis
    for point1 in end_points:
        x1 = point1[0]
        y1 = point1[1]
        for point2 in end_points:
            if point2 != point1:
                x2 = point2[0]
                y2 = point2[1]

                if point1 not in same_y_1 and point1 not in same_y_2:
                    if y1 - find_points_threshold <= y2 <= y1 + find_points_threshold:
                        if tb3.pos.y < y2:
                            same_x_1.append(point2)
                        else:
                            same_x_2.append(point2)

                if point1 not in same_x_1 and point1 not in same_x_2:
                    if x1 - find_points_threshold <= x2 <= x1 + find_points_threshold:
                        if tb3.pos.x < x2:
                            same_y_1.append(point2)
                        else:
                            same_y_2.append(point2)

    if len(same_x_1) > wall_threshold:
        wall += 1
    if len(same_x_2) > wall_threshold:
        wall += 1
    if len(same_y_1) > wall_threshold:
        wall += 1
    if len(same_y_2) > wall_threshold:
        wall += 1

    if wall >= 3:
        print("DEAD END: TRUE")
    else:
        print("DEAD END: FALSE")

    if visualize:
        visualize_endpoints(end_points, "Endpoints")
        visualize_endpoints(same_y_1, "SAME Y1")
        visualize_endpoints(same_y_2, "SAME Y2")
        visualize_endpoints(same_x_1, "SAME X1")
        visualize_endpoints(same_x_2, "SAME X2")

def visualize_endpoints(points, name):
    """
    Helper function to visualize the laser points
    :param points: poiint cloud which should be visualized ([x,y],..)
    :param name: Name for the plot window
    """
    only_x = []
    only_y = []

    for point in points:
        only_x.append(point[0])
        only_y.append(point[1])

    plt.title(name)
    plt.scatter(only_x, only_y)
    plt.show()
