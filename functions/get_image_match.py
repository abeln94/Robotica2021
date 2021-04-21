# Standard imports
import cv2
import numpy as np

# Project imports
from classes import Cfg

Cfg.add_argument("-match", "--image_match", help="Show the camera in the robot", action="store_true")

# ---------------------------------------------------------------------------------------------------
# CONFIGURATION

# ASCI codes to interact with windows when debug
ESC = 27

# max number of features to extract per image
MAX_FEATURES = 500
# REQUIRED number of correspondences (matches) found:
MIN_MATCH_COUNT = 20  # initially
MIN_MATCH_OBJECTFOUND = 15  # after robust check, to consider object-found


# ---------------------------------------------------------------------------------------------------
# FUNCTIONS

def match_images(image, capture):
    """
    Returns whether image is contained into capture, and if it is, returns the 
    2D coordinates where the blob is found
    :param image: image to find into capture
    :param capture: a given image
    """

    # If figure not found, this pair is returned
    NOT_FOUND = False, None

    # Feature extractor uses grayscale images
    img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    cap = cv2.cvtColor(capture, cv2.COLOR_BGR2GRAY)

    # Create a detector with the parameters
    # CURRENT RASPBERRY opencv version is 3.4.13
    # Initiate BRISK detector --> you could use any other detector, including NON binary features (SIFT, SURF)
    # but this is the best performing one in this version
    detector = cv2.BRISK_create()

    # find the keypoints and corresponding descriptors
    kpImg, desImg = detector.detectAndCompute(img, None)
    kpCap, desCap = detector.detectAndCompute(cap, None)

    if desImg is None or desCap is None:
        # WARNING: empty detection?
        return NOT_FOUND
    if len(desImg) < MIN_MATCH_COUNT or len(desCap) < MIN_MATCH_COUNT:
        # WARNING: not enough FEATURES (im1: len(desImg), im2: len(desCap))
        return NOT_FOUND

    # If binary features are used
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(desImg, desCap)
    goodKeypoints = sorted(matches, key=lambda x: x.distance)

    # Show matches if requested
    if Cfg.image_match:
        img_tmp = cv2.drawMatches(image, kpImg, capture, kpCap, goodKeypoints, None)
        cv2.imshow("Matches", img_tmp)

    # If enough matches found, figure is considered to be recognized
    if len(goodKeypoints) > MIN_MATCH_COUNT:
        img_pts = np.float32([kpImg[m.queryIdx].pt for m in goodKeypoints]).reshape(-1, 1, 2)
        cap_pts = np.float32([kpCap[m.trainIdx].pt for m in goodKeypoints]).reshape(-1, 1, 2)
        H_21, mask = cv2.findHomography(img_pts, cap_pts, cv2.RANSAC, 3.0)
        matchesMask = mask.ravel().tolist()
        num_robust_matches = np.sum(matchesMask)
        if num_robust_matches < MIN_MATCH_OBJECTFOUND:
            # NOT enough ROBUST matches found - num_robust_matches (required MIN_MATCH_OBJECTFOUND)
            return NOT_FOUND
        h, w = img.shape
        pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)
        box_corners = cv2.perspectiveTransform(pts, H_21)
        # cap_res = cv2.polylines(capture, [np.int32(box_corners)], True,
        #                          color=(255, 255, 255), thickness=3)

        # Show matches if requested
        if Cfg.image_match:
            draw_params = dict(matchColor=(0, 255, 0),  # draw matches in green color
                               singlePointColor=None,
                               matchesMask=matchesMask,  # draw only inliers
                               flags=2)
            img3 = cv2.drawMatches(image, kpImg, capture, kpCap, goodKeypoints, None, **draw_params)
            cv2.imshow("Matches", img3)
        # ROBUST matches found - np.sum(matchesMask) (out of len(goodKeypoints)) --> OBJECT FOUND"
        sumOfColumns = sum(box_corners, 0)[0]
        xCenter = sumOfColumns[0] / len(box_corners)
        yCenter = sumOfColumns[1] / len(box_corners)
        return True, (xCenter / Cfg.CAMERA_WIDTH, yCenter / Cfg.CAMERA_HEIGHT)
    else:
        # Not enough initial matches are found - len(goodKeypoints) (required MIN_MATCH_COUNT)"
        return NOT_FOUND
