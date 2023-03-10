import math
import itertools
import os
import random
from statistics import mean, median
import sys

from matplotlib import pyplot as plt

from utils.tb3_math import rad


def get_red_beam(tb3):
    reds = [x for x in range(0, len(tb3.beams)) if tb3.beam_intensities[x] == 2]
    med = int(median(reds))
    tb3.red_beam = med
    return (med, tb3.beams[med])


def detect_red_with_lds(tb3):
    return any(tb3.beam_intensities[x] == 2 for x in range(0, len(tb3.beams)))


def detect_red_with_lds_front(tb3):
    return all(tb3.beam_intensities[x] == 2 for x in range(-20, 20))


def search_object(tb3: object, laser):
    """
    Search for a object with the given laser in the specifiy scan_range.

    :param tb3: Bot object.
    :param laser: Array of all laser data.

    # 60 - 120 right side
    # 150 -210 behind
    # 240 - 300 left
    # -30 - 30 front

    Set state to "object".
    """

    if tb3.front_search:
        tb3.max_dist_front = laser[0]
        if tb3.min_dist_front >= laser[0]:
            tb3.object_front = True
        else:
            tb3.object_front = False

    if tb3.back_search:
        tb3.max_dist_back = laser[180]
        if tb3.min_dist_back >= laser[180]:
            tb3.object_back = True
        else:
            tb3.object_back = False

    if tb3.right_search:
        tb3.max_dist_right = laser[-90]
        if tb3.min_dist_right >= laser[-90]:
            tb3.object_right = True
        else:
            tb3.object_right = False

    if tb3.left_search:
        tb3.max_dist_left = laser[90]
        if tb3.min_dist_left >= laser[90]:
            tb3.object_left = True
        else:
            tb3.object_left = False


def check_front_wall(tb3, end=False):
    if end:
        return any(tb3.beams[x] < 0.15 for x in range(-30, 30))
    return any(tb3.beams[x] < tb3.front_distance for x in range(-30, 30))


def check_back_wall(tb3):
    return any(tb3.beams[x] < tb3.back_distance for x in range(150, 210))


def check_left_wall(tb3):
    return any(tb3.beams[x] < tb3.left_distance for x in range(240, 300))


def check_right_wall(tb3):
    return any(tb3.beams[x] < tb3.right_distance for x in range(60, 120))


def get_grouped_beams(tb3: object, beams):
    """

    :param tb3:
    :param op_beams:
    :return:
    """
    op_beams = [(x, beams[x]) for x in range(0, len(beams)) if tb3.beam_distance < beams[x] < tb3.max_beam_distance]
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


def get_grouped(tb3):
    if len(tb3.op_beams) == 0:
        return
    tb3.groups = [[tb3.op_beams[0][0]]]
    idx = tb3.op_beams[0][0]
    for x in tb3.op_beams[1:]:
        id = x[0]
        ab = abs(idx - id)
        if ab == 1:
            idx = id
            tb3.groups[-1].append(id)
        else:
            idx = id
            tb3.groups.append([id])
    filt_groups = list(filter(lambda x: not check_dead_end(tb3, shorten_group(x)) and big_enough_group(x), tb3.groups))
    if len(filt_groups) >= 2 and filt_groups[0][0] == 0 and filt_groups[-1][-1] == 359:
        filt_groups = [filt_groups[-1] + filt_groups[0]] + filt_groups[1:-1]
    if len(filt_groups) >= 1:
        tb3.groups = filt_groups


def big_enough_group(group):
    return len(group) > 25


def shorten_group(group):
    return (filter(lambda x: x < 2, group))


def get_degree_of_random_group(tb3):
    if len(tb3.groups) >= 1:
        rand_group = tb3.groups[0]
        med = rand_group[int(len(rand_group) / 2)]
        return (med, tb3.beams[med])


def get_degree_of_prefered_group(tb3):
    saved_g = []
    highest = 0
    for g in tb3.groups:
        for x in g:
            if tb3.beams[x] > highest:
                saved_g = g
                highest = tb3.beams[x]
    med = int(median(saved_g))
    if tb3.last_origin_degree is None:
        return (med, tb3.beams[med])
    if tb3.last_origin_degree - 15 < med < tb3.last_origin_degree + 15:
        # Forbidden direction, go to next group
        if len(tb3.groups) == 1:
            tb3.state = 4
            return

def get_degree_of_group(tb3, beam_group):
    med = beam_group[int(len(beam_group) / 2)]
    return (med, tb3.beams[med])


def drive_through_maze(tb3):
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
    return point


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


def check_dead_end(tb3, beam_group, find_points_threshold=0.01, wall_threshold=5, visualize=False):
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
        end_point = get_laser_endpoint(tb3.pos.x, tb3.pos.y, tb3.beams[beam], beam)
        x_axis.append(end_point[0])
        y_axis.append(end_point[1])
        end_points.append(end_point)
        i += 1
        # print(i)

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

    if check_wall(same_x_1, wall_threshold):
        wall += 1
    if check_wall(same_x_2, wall_threshold):
        wall += 1
    if check_wall(same_y_1, wall_threshold):
        wall += 1
    if check_wall(same_y_2, wall_threshold):
        wall += 1

    if visualize:
        visualize_endpoints(end_points, "Endpoints")
        visualize_endpoints(same_y_1, "SAME Y1")
        visualize_endpoints(same_y_2, "SAME Y2")
        visualize_endpoints(same_x_1, "SAME X1")
        visualize_endpoints(same_x_2, "SAME X2")

    if wall == 3:
        # map(list, set(map(tuple, same_x_1)))
        if correct_wall_formation(
                list(k for k, _ in itertools.groupby(same_x_1)),
                list(k for k, _ in itertools.groupby(same_y_1)),
                list(k for k, _ in itertools.groupby(same_x_2)),
                list(k for k, _ in itertools.groupby(same_y_2)),
        ):
            tb3.deadend = True
            return True
    tb3.deadend = False
    return False


def check_wall(wall, thresh=5):
    return len(wall) > thresh


def correct_wall_formation(walls_x_1, walls_y_1, walls_x_2, walls_y_2):
    if check_wall(walls_x_1) and check_wall(walls_x_2) and check_wall(walls_y_1):
        x1 = mean(map(lambda x: x[1], walls_x_1))
        x2 = mean(map(lambda x: x[1], walls_x_2))
        thresh = len(walls_y_1) * 0.9
        if x1 < x2:
            return len([y for y in walls_y_1 if x1 <= y[1] <= x2]) >= thresh
        else:
            return len([y for y in walls_y_1 if x2 <= y[1] <= x1]) >= thresh
    elif check_wall(walls_x_1) and check_wall(walls_x_2) and check_wall(walls_y_2):
        x1 = mean(map(lambda x: x[1], walls_x_1))
        x2 = mean(map(lambda x: x[1], walls_x_2))
        thresh = len(walls_y_2) * 0.9
        if x1 < x2:
            return len([y for y in walls_y_2 if x1 <= y[1] <= x2]) >= thresh
        else:
            return len([y for y in walls_y_2 if x2 <= y[1] <= x1]) >= thresh
    elif check_wall(walls_y_1) and check_wall(walls_y_2) and check_wall(walls_x_1):
        y1 = mean(map(lambda y: y[0], walls_y_1))
        y2 = mean(map(lambda y: y[0], walls_y_2))
        thresh = len(walls_x_1) * 0.9
        if y1 < y2:
            return len([x for x in walls_x_1 if y1 <= x[0] <= y2]) >= thresh
        else:
            return len([x for x in walls_x_1 if y2 <= x[0] <= y1]) >= thresh
    elif check_wall(walls_y_1) and check_wall(walls_y_2) and check_wall(walls_x_2):
        y1 = mean(map(lambda y: y[0], walls_y_1))
        y2 = mean(map(lambda y: y[0], walls_y_2))
        thresh = len(walls_x_2) * 0.9
        if y1 < y2:
            return len([x for x in walls_x_2 if y1 <= x[0] <= y2]) >= thresh
        else:
            return len([x for x in walls_x_2 if y2 <= x[0] <= y1]) >= thresh

    # print(f"\n\n")
    # if len(walls_x) < len(walls_y):
    #     # Compare if walls_x is in between walls_y
    #     y1 = mean(map(lambda y: y[0], walls_y[0]))
    #     y2 = mean(map(lambda y: y[0], walls_y[1]))
    #     if y1 < y2:
    #         print(f"x < y || y1 < y2 || {len([x for x in walls_x[0] if y1 <= x[0] <= y2])}")
    #         return len([x for x in walls_x[0] if y1 <= x[0] <= y2]) >= thresh
    #     else:
    #         print(f"x{len(walls_x)} < y{len(walls_y)} || y1:{y1} > y2:{y2} | {len(walls_x[0])} | {len([x for x in walls_x[0] if y2 <= x[0] <= y1])}")
    #         return len([x for x in walls_x[0] if y2 <= x[0] <= y1]) >= thresh
    # else:
    #     # Compare if walls_y is in between walls_x
    #     x1 = mean(map(lambda x: x[1], walls_x[0]))
    #     x2 = mean(map(lambda x: x[1], walls_x[1]))
    #     if x1 < x2:
    #         print(f"x > y || x1 < x2 || {len([y for y in walls_y[0] if x1 <= y[1] <= x2])}")
    #         return len([y for y in walls_y[0] if x1 <= y[1] <= x2]) >= thresh
    #     else:
    #         print(f"x > y || x1 > x2 || {len([y for y in walls_y[0] if x2 <= y[1] <= x1])}")
    #         return len([y for y in walls_y[0] if x2 <= y[1] <= x1]) >= thresh
