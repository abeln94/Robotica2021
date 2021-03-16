"""
Simply display the contents of the webcam with optional mirroring using OpenCV
via the new Pythonic cv2 interface.  Press <esc> to quit.
"""

import cv2

# Setup default values for SimpleBlobDetector parameters.

params = cv2.SimpleBlobDetector_Params()

# These are just examples, tune your own if needed
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
# params.blobColor = 0
params.filterByConvexity = False
params.filterByInertia = False

# Create a detector with the parameters
ver = cv2.__version__.split('.')
if int(ver[0]) < 3:
    detector = cv2.SimpleBlobDetector(params)
else:
    detector = cv2.SimpleBlobDetector_create(params)

cam = cv2.VideoCapture(0)

while True:
    ret_val, img = cam.read()
    if not ret_val: continue

    # then the result is blurred
    img = cv2.GaussianBlur(img, (9, 9), 3, 3)

    # converting the input stream into HSV color space
    img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # because hue wraps up and to extract as many "red objects" as possible, I define lower and upper boundaries for brighter and for darker red shades
    # after masking the red shades out, I add the two images
    img = cv2.addWeighted(
        cv2.inRange(img, (0, 100, 100), (10, 255, 255)), 1.0,
        cv2.inRange(img, (350, 100, 100), (360, 255, 255)), 1.0,
        0.0)

    # some morphological operations (closing) to remove small blobs
    img = cv2.erode(img, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
    img = cv2.dilate(img, cv2.getStructuringElement(cv2.MORPH_RECT, (8, 8)))

    # on the color-masked, blurred and morphed image I apply the cv2.HoughCircles-method to detect circle-shaped objects
    detected_circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1, 150, param1=100, param2=20, minRadius=20, maxRadius=200)
    if detected_circles is not None:
        for circle in detected_circles[0, :]:
            circled_orig = cv2.circle(img, (circle[0], circle[1]), int(circle[2]), (0, 255, 0), thickness=3)
        cv2.imshow("original", circled_orig)
    else:
        cv2.imshow("original", img)
    #
    # img_HSV = cv2.cvtColor(img_BGR, cv2.COLOR_BGR2HSV)
    #
    # cv2.inRange()
    #
    # # BY DEFAULT, opencv IMAGES have BGR format
    # redMin = (10, 10, 100)
    # redMax = (50, 50, 255)
    #
    # blueMin = (60, 10, 10)
    # blueMax = (255, 100, 100)
    #
    # mask_red = cv2.inRange(img_BGR, redMin, redMax)
    # mask_blue = cv2.inRange(img_BGR, blueMin, blueMax)
    #
    # # apply the mask
    # red = cv2.bitwise_and(img_BGR, img_BGR, mask=mask_red)
    # blue = cv2.bitwise_and(img_BGR, img_BGR, mask=mask_blue)
    # # show resulting filtered image next to the original one
    # cv2.imshow("Red regions", np.hstack([img_BGR, red]))
    # cv2.imshow("Blue regions", np.hstack([img_BGR, blue]))
    #
    # # detector finds "dark" blobs by default, so invert image for results with same detector
    # keypoints_red = detector.detect(255 - mask_red)
    # keypoints_blue = detector.detect(255 - mask_blue)
    #
    # # documentation of SimpleBlobDetector is not clear on what kp.size is exactly, but it looks like the diameter of the blob.
    # # for kp in keypoints_red:
    # #     print(kp.pt[0], kp.pt[1], kp.size)
    #
    # # Draw detected blobs as red circles.
    # # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures
    # # the size of the circle corresponds to the size of blob
    # im_with_keypoints = cv2.drawKeypoints(img_BGR, keypoints_red, np.array([]),
    #                                       (255, 255, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    # im_with_keypoints2 = cv2.drawKeypoints(img_BGR, keypoints_blue, np.array([]),
    #                                        (255, 255, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    #
    # # Show mask and blobs found
    # cv2.imshow("Keypoints on RED", im_with_keypoints)
    # cv2.imshow("Keypoints on BLUE", im_with_keypoints2)

    if cv2.waitKey(1) == 27:
        break  # esc to quit
cv2.destroyAllWindows()
