def stop(tb3: object):
    """
    Stop the bot set velocity and rotation to 0.
    And stop scan search.

    Set state to "stop".
    """
    tb3.vel(0, 0)
    tb3.front_search = False
    tb3.back_search = False
    tb3.right_search = False
    tb3.left_search = False
    print("Bot stopped")
    tb3.state = "stop"


def drive(tb3: object, velocity: int):
    """
    Drive the bot with given velocity.
    :param tb3: The bot which should drive
    :param velocity: integer Velocity is given in percentages. Positiv Integer drive forward, negative integer drive backward

    Set state to "drive".
    """
    if velocity == 0:
        tb3.stop()
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
        tb3.stop()
        return
    tb3.vel(0, rotation_velocity)
    tb3.state = "rotate"
    print("Bot is rotating to the left" if rotation_velocity > 0 else "Bot is rotating to the right side")


def search_object(tb3: object, laser, scan_range_front=0, scan_range_back=0, scan_range_left=0, scan_range_right=0):
    """
    Search for a object with the given laser in the specifiy scan_range.

    :param scan_range_right:
    :param scan_range_left:
    :param tb3: Bot object.
    :param laser:
    :param scan_range_front:
    :param scan_range_back:

    # 60 - 120 right side
    # 150 -210 behind
    # 240 - 300 left
    # -30 - 30 front

    Set state to "object".
    """

    if tb3.front_search:
        if scan_range_front >= laser[0]:
            tb3.state = "object_front"
            object_position = "front"
            print("Attention! Object at {}.".format(object_position))
        else:
            print("No Object in front.")

    elif tb3.back_search:
        if scan_range_back >= laser[180]:
            tb3.state = "object_back"
            object_position = "back"
            print("Attention! Object at {}.".format(object_position))
        else:
            print("No Object in behind.")

    elif tb3.right_search:
        if scan_range_right >= laser[-90]:
            tb3.state = "object_right"
            object_position = "right"
            print("Attention! Object at {}.".format(object_position))
        else:
            print("No Object in right.")

    elif tb3.left_search:
        if scan_range_left >= laser[90]:
            tb3.state = "object_left"
            object_position = "left"
            print("Attention! Object at {}.".format(object_position))
        else:
            print("No Object in left.")

