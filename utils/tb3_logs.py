import math
import os
from utils.tb3_lds_laser import *

def diagnostics_final(tb3):
    os.system("clear")
    try:
        if tb3.pos is not None and tb3.orient is not None:
            if hasattr(tb3, 'odom_sub'):
                print(
                    f"X{' ' * len(str(tb3.pos.x))}\tY{' ' * len(str(tb3.pos.y))}\tZ{' ' * len(str(tb3.pos.z))}\t|\tRotX{' ' * len(str(tb3.orient[0]))}\tRotY{' ' * (len(str(tb3.orient[1])) - 3)}\tRotZ")
                print(f"{tb3.pos.x}\t{tb3.pos.y}\t{tb3.pos.z}\t|\t{tb3.orient[0]}\t{tb3.orient[1]}\t{tb3.orient[2]}")


            print(f"Bot in state: {tb3.state.name}")
            print(f"Number of beams: {len(tb3.beams)}")
            print(f"Number of beam-intensities: {len(tb3.beam_intensities)}")
            print(f"\n\nCurrent Angle: {tb3.orient[0]} :: {tb3.angle} >> Difference {abs(tb3.orient[0] - tb3.angle)}")
            print(f"Init angle: {tb3.init_angle}")

    except Exception as e:
        print(e)

def diagnostics(tb3):
    os.system("clear")
    # print(f"{get_title()}")
    try:
        if tb3.pos is not None and tb3.orient is not None:
            if hasattr(tb3, 'odom_sub'):
                print(
                    f"X{' ' * len(str(tb3.pos.x))}\tY{' ' * len(str(tb3.pos.y))}\tZ{' ' * len(str(tb3.pos.z))}\t|\tRotX{' ' * len(str(tb3.orient[0]))}\tRotY{' ' * (len(str(tb3.orient[1])) - 3)}\tRotZ")
                print(f"{tb3.pos.x}\t{tb3.pos.y}\t{tb3.pos.z}\t|\t{tb3.orient[0]}\t{tb3.orient[1]}\t{tb3.orient[2]}")

            print("\n***OBJECT VIEW***")
            print(f"FRONT| Object in range: {tb3.object_front} | Detection distance: {tb3.min_dist_front} | Object at distance: {tb3.max_dist_front}")
            print(f"LEFT | Object in range: {tb3.object_left}  | Detection distance: {tb3.min_dist_left}  | Object at distance: {tb3.max_dist_left}")
            print(f"BACK | Object in range: {tb3.object_back}  | Detection distance: {tb3.min_dist_back}  | Object at distance: {tb3.max_dist_back}")
            print(f"RIGHT| Object in range: {tb3.object_right} | Detection distance: {tb3.min_dist_right} | Object at distance: {tb3.max_dist_right}")

            print("\n***COMPASS VIEW***")
            print(f"View: {tb3.VIEW}")
            print(f"Rotating: {tb3.rot}")
            if tb3.rotate_direction is None:
                rotate_direction = "-"
            elif tb3.rotate_direction < 0:
                rotate_direction = "right"
            elif tb3.rotate_direction > 0:
                rotate_direction = "left"
            else:
                rotate_direction = "-"
            print(f"Rotate direction:{rotate_direction}")

            print("\n***COLOR DETECTION***")
            print(f"Detected red wall: {tb3.color}")

    except Exception as e:
        print(e)


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
    '   :  ;|  ' | |  . .'  :  /  :__,'| :  |  | :    /    /  ||   : '  |'   | |: ::__,'| :                 \  \   \\
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
    if state == -2:
        return """
   ___                       _   ___       __                    _   _          
  / __|___ _ _  ___ _ _ __ _| | |_ _|_ _  / _|___ _ _ _ __  __ _| |_(_)___ _ _  
 | (_ / -_) ' \/ -_) '_/ _` | |  | || ' \|  _/ _ \ '_| '  \/ _` |  _| / _ \ ' \ 
  \___\___|_||_\___|_| \__,_|_| |___|_||_|_| \___/_| |_|_|_\__,_|\__|_\___/_||_|
  """
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
  ___ _        _         _ _  _   ___      _        _                         _
 / __| |_ __ _| |_ ___  | | |(_) | _ ) ___| |_   __| |_ ___ _ __ _ __  ___ __| |
 \__ \  _/ _` |  _/ -_) |_  _)_  | _ \/ _ \  _| (_-<  _/ _ \ '_ \ '_ \/ -_) _` |
 |___/\__\__,_|\__\___)   |_|(_) |___/\___/\__| /__/\__\___/ .__/ .__/\___\__,_|
                                                           |_|  |_|
    """
    elif state == 5:
        return """
  ___ _        _         ___ _   ___        _    __                  _                _ _ _ _ 
 / __| |_ __ _| |_ ___  | __(_) | _ \___ __| |  / _|___ _  _ _ _  __| |    __ _ ___  | | | | |
 \__ \  _/ _` |  _/ -_) |__ \_  |   / -_) _` | |  _/ _ \ || | ' \/ _` |_  / _` / _ \ |_|_|_|_|
 |___/\__\__,_|\__\___| |___(_) |_|_\___\__,_| |_| \___/\_,_|_||_\__,_( ) \__, \___/ (_|_|_|_)
                                                                      |/  |___/               
    """
    elif state == 6:
        return """
  ___ _        _          __ _  __      _____  __      _____  _  _   _ _ _ _ _ 
 / __| |_ __ _| |_ ___   / /(_) \ \    / / __| \ \    / / _ \| \| | | | | | | |
 \__ \  _/ _` |  _/ -_) / _ \_   \ \/\/ /| _|   \ \/\/ / (_) | .` | |_|_|_|_|_|
 |___/\__\__,_|\__\___| \___(_)   \_/\_/ |___|   \_/\_/ \___/|_|\_| (_|_|_|_|_)
    """
    else:
        return ""
