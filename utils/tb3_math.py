import math


def rad(deg):
    """
    Convert degree to radians
    :param deg: degree value
    :return: converted radian
    """
    return math.radians(deg)

def rad_overlap(deg):
    if deg > 180:
        deg -= 360
    if deg < -180:
        deg += 360
    return math.radians(deg)