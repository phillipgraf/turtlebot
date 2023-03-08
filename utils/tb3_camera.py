import numpy as np
import cv2


def detect_red(tb3: object, fill_percentage=0.001, red_in_center=False):
    """
    Detect the color red of an object in the front between the rgb ranges:
    lower: 0, 0, 120
    upper: 10, 10, 130

    Set color to "red".  If no red empty string "".

    :param red_in_center:
    :param fill_percentage:
    :param tb3: Bot object
    """
    lower_red = np.array([0, 0, 100], np.uint8)
    upper_red = np.array([10, 10, 180], np.uint8)
    mask = cv2.inRange(tb3.image, lower_red, upper_red)
    tb3.red_percentage = (mask == 255).sum() / (tb3.image.shape[0] * tb3.image.shape[1])
    needed_pxl = tb3.image.shape[0] * tb3.image.shape[1] * fill_percentage
    tb3.needed_pxl = needed_pxl
    if (mask == 255).sum() > needed_pxl:
        tb3.img_shape = mask.shape
        tb3.color = "red"
        # if red_in_center:
        #     if (mask.T[:5] != 255).sum() == 0 and (mask.T[-5:] != 255).sum() == 0:
        #         return True
        #     else:
        #         return False
        return True
    else:
        return False
        tb3.color = ""


def start_video(tb3: object):
    """
    Show a video of the camera view.
    :param tb3: Bot object.
    """
    cv2.imshow("Video", tb3.image)
    if cv2.waitKey(10) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
