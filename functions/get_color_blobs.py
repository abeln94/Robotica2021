# Standard imports
import cv2
import numpy as np

from classes import Cfg

Cfg.add_argument("-blobs", "--color_blobs", help="Show the camera in the robot", action="store_true")

# Setup default values for SimpleBlobDetector parameters.
params = cv2.SimpleBlobDetector_Params()

# These are just examples, tune your own if needed
# Change thresholds
params.minThreshold = 10
params.maxThreshold = 200

# Filter by Area
params.filterByArea = False
params.minArea = 150 * 4
params.maxArea = 50000 * 4

# Filter by Circularity
params.filterByCircularity = False
params.minCircularity = 0.1

# Filter by Color
params.filterByColor = True
# not directly color, but intensity on the channel input
params.blobColor = 255
params.filterByConvexity = False
params.filterByInertia = False

# Create a detector with the parameters
ver = cv2.__version__.split('.')
if int(ver[0]) < 3:
    detector = cv2.SimpleBlobDetector(params)
else:
    detector = cv2.SimpleBlobDetector_create(params)


def get_color_blobs(img_BGR, rangeMin=(160, 80, 50), rangeMax=(10, 255, 255)):
    """
    Returns the detected blobs of a BGR image between an range of HSV color
    :param img_BGR: The BGR image to apply the detector
    :param rangeMin: The min HSV value to be detected
    :param rangeMax: The max HSV value to be detected
    """
    # keypoints on original image (will look for blobs in grayscale)
    image = cv2.cvtColor(img_BGR, cv2.COLOR_BGR2HSV)

    # HSV FORMAT ranges
    if rangeMin[0] > rangeMax[0]:
        rangeMin1 = rangeMin
        rangeMax1 = (179, rangeMax[1], rangeMax[2])

        rangeMin2 = (0, rangeMin[1], rangeMin[2])
        rangeMax2 = rangeMax
    else:
        rangeMin1 = rangeMin
        rangeMax1 = rangeMin

        rangeMin2 = rangeMin
        rangeMax2 = rangeMax

    # Obtain the mask
    mask1 = cv2.inRange(image, rangeMin1, rangeMax1)
    mask2 = cv2.inRange(image, rangeMin2, rangeMax2)
    mask = mask1 | mask2

    # some morphological operations (closing) to remove small blobs
    mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (8, 8)))
    mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10)))

    if cv2.countNonZero(mask) == 0:
        # nothing to search
        return []

    # apply the mask
    keypoints = detector.detect(mask)
    # keypoints = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1, 150, param1=100, param2=20, minRadius=20, maxRadius=200)

    if Cfg.color_blobs:
        regions = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)

        for kp in keypoints:
            print("Keypoint:", kp.pt[0], kp.pt[1], kp.size)

        # Show mask and blobs found
        im_with_keypoints = cv2.drawKeypoints(img_BGR, keypoints, np.array([]), (255, 255, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        cv2.imshow("Detected Regions", cv2.flip(np.hstack([regions, im_with_keypoints]), -1))
        cv2.waitKey(1)

    return keypoints


def get_blob(img_BGR, rangeMin=(160, 80, 50), rangeMax=(10, 255, 255)):
    """
    Returns the most promising blob (the bigger) of a BGR image between an range of HSV color
    :param img_BGR: The BGR image to apply the detector
    :param rangeMin: The min HSV value to be detected
    :param rangeMax: The max HSV value to be detected
    """
    blobs = get_color_blobs(img_BGR, rangeMin, rangeMax)
    blob = max(blobs, default=None, key=lambda item: item.size)

    return (blob.pt[0] / Cfg.CAMERA_WIDTH, blob.pt[1] / Cfg.CAMERA_HEIGHT) if blob is not None else None


def position_reached(img_BGR, rangeMin=(160, 80, 50), rangeMax=(10, 255, 255)):
    """
    Returns if the robot has reached the position to catch the ball
        given an image and the range of colours of the ball
    :param img_BGR: The BGR image to apply the detector
    :param rangeMin: The min HSV value to be detected
    :param rangeMax: The max HSV value to be detected
    """
    image = cv2.cvtColor(img_BGR, cv2.COLOR_BGR2HSV)
    if rangeMin[0] > rangeMax[0]:
        rangeMin1 = rangeMin
        rangeMax1 = (179, rangeMax[1], rangeMax[2])

        rangeMin2 = (0, rangeMin[1], rangeMin[2])
        rangeMax2 = rangeMax
    else:
        rangeMin1 = rangeMin
        rangeMax1 = rangeMin

        rangeMin2 = rangeMin
        rangeMax2 = rangeMax

    # Obtain the mask
    mask1 = cv2.inRange(image, rangeMin1, rangeMax1)
    mask2 = cv2.inRange(image, rangeMin2, rangeMax2)
    mask = mask1 | mask2

    PX = np.floor(Cfg.CAMERA_HEIGHT / 4.8)
    return cv2.countNonZero(mask[0:PX, :]) / Cfg.CAMERA_WIDTH / PX > 0.6
