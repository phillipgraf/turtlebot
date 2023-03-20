import math
import itertools
import os
import random
from statistics import mean, median
import sys

from matplotlib import pyplot as plt

from utils.tb3_math import *

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

def collide_with_wall(tb3):
    """
    Collide with the wall for a given range
    :param tb3: Bot object
    """
    return any(tb3.beams[x] < 0.15 for x in range(-30, 30))






