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
MIN_MATCH_COUNT=20          # initially
MIN_MATCH_OBJECTFOUND=15    # after robust check, to consider object-found

#---------------------------------------------------------------------------------------------------
# FUNCTIONS

def match_images(img1_bgr, img2_bgr):
    """
    Returns whether img1_bgr is contained into img2_bgr, and if it is, returns the 
    2D coordinates where the blob is found
    :param img1_bgr: image to find into img2_bgr
    :param img2_bgr: a given image
    """

    # If figure not found, this pair is returned
    NOT_FOUND = False, None

    # Feature extractor uses grayscale images
    img1 = cv2.cvtColor(img1_bgr, cv2.COLOR_BGR2GRAY)
    img2 = cv2.cvtColor(img2_bgr, cv2.COLOR_BGR2GRAY)
    
    # Create a detector with the parameters
    # CURRENT RASPBERRY opencv version is 3.4.13
    # Initiate BRISK detector --> you could use any other detector, including NON binary features (SIFT, SURF)
    # but this is the best performing one in this version
    detector = cv2.BRISK_create()

    # find the keypoints and corresponding descriptors
    kp1, des1 = detector.detectAndCompute(img1,None)
    kp2, des2 = detector.detectAndCompute(img2,None)

    if des1 is None or des2 is None:
        # WARNING: empty detection?
        return NOT_FOUND
    if len(des1) < MIN_MATCH_COUNT or len(des2) < MIN_MATCH_COUNT:
        # WARNING: not enough FEATURES (im1: len(des1), im2: len(des2))
        return NOT_FOUND

    # If binary features are used
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = bf.match(des1,des2)
    good = sorted(matches, key = lambda x:x.distance)

    # Show matches if requested
    if Cfg.image_match:
        img_tmp = cv2.drawMatches(img1_bgr, kp1, img2_bgr, kp2, good, None)
        cv2.imshow("All matches", img_tmp)

    # If enough matches found, figure is considered to be recognized
    if len(good) > MIN_MATCH_COUNT:
        src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
        H_21, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 3.0)
        matchesMask = mask.ravel().tolist()
        num_robust_matches = np.sum(matchesMask)
        if num_robust_matches < MIN_MATCH_OBJECTFOUND:
            # NOT enough ROBUST matches found - num_robust_matches (required MIN_MATCH_OBJECTFOUND)
            return NOT_FOUND
        h,w = img1.shape
        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
        dst = cv2.perspectiveTransform(pts,H_21)
        img2_res = cv2.polylines(img2_bgr, [np.int32(dst)], True, 
                                 color=(255,255,255), thickness=3)
        # Show matches if requested
        if Cfg.image_match:
            draw_params = dict(matchColor=(0, 255, 0),  # draw matches in green color
                               singlePointColor=None,
                               matchesMask=matchesMask,  # draw only inliers
                               flags=2)
            img3 = cv2.drawMatches(img1_bgr, kp1, img2_bgr, kp2, good, None, **draw_params)
            cv2.imshow("INLIERS", img3)
        # ROBUST matches found - np.sum(matchesMask) (out of len(good)) --> OBJECT FOUND"
        sumOfColumns = sum(dst,0)[0]
        xCenter = sumOfColumns[0] / len(dst)
        yCenter = sumOfColumns[1] / len(dst)
        return True, (xCenter / Cfg.CAMERA_WIDTH, yCenter / Cfg.CAMERA_HEIGHT)
    else:
        # Not enough initial matches are found - len(good) (required MIN_MATCH_COUNT)"
        return NOT_FOUND
