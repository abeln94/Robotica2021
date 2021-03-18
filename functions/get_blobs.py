#!/usr/bin/python
# -*- coding: utf-8 -*-


# Standard imports
import cv2
import numpy as np;

def get_blobs(img_BGR):
    # Aditional params
    hsv = True
    plot_result = False

    image = cv2.cvtColor(img_BGR,cv2.COLOR_BGR2HSV)
    colorRange1Min = (0, 70, 50)
    colorRange1Max = (5, 255, 255)

    colorRange2Min = (170, 70, 50)
    colorRange2Max = (180, 255, 255)

    params = cv2.SimpleBlobDetector_Params()

    # Change thresholds
    params.minThreshold = 10
    params.maxThreshold = 200

    # Filter by Area
    params.filterByArea = True
    params.minArea = 200
    params.maxArea = 10000

    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.1

    # Filter by Color
    params.filterByColor = False
    # not directly color, but intensity on the channel input
    #params.blobColor = 0
    params.filterByConvexity = False
    params.filterByInertia = False

    # Create a detector with the parameters
    ver = (cv2.__version__).split('.')
    if int(ver[0]) < 3 :
        detector = cv2.SimpleBlobDetector(params)
    else :
        detector = cv2.SimpleBlobDetector_create(params)

    mask1 = cv2.inRange(image, colorRange1Min, colorRange1Max)
    mask2 = cv2.inRange(image, colorRange2Min, colorRange2Max)
    mask = mask1 | mask2
    #mask = mask2

    cv2.imshow("mask", mask)

    keypoints = detector.detect(255-mask)

    im_with_keypoints = cv2.drawKeypoints(img_BGR, keypoints, np.array([]),
        (255,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    if plot_result:
        regions = cv2.bitwise_and(image, image, mask = mask)
        regions = cv2.cvtColor(regions,cv2.COLOR_HSV2BGR)
        cv2.imshow("Detected Regions", np.hstack([img_BGR, regions]))

        for kp in keypoints:
	        print(kp.pt[0], kp.pt[1], kp.size)

        # Show mask and blobs found
        cv2.imshow("Keypoints on RED", im_with_keypoints)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return im_with_keypoints