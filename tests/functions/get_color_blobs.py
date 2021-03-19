# Standard imports
import cv2
import numpy as np

# Setup default values for SimpleBlobDetector parameters.
params = cv2.SimpleBlobDetector_Params()

# These are just examples, tune your own if needed
# Change thresholds
params.minThreshold = 10
params.maxThreshold = 200

# Filter by Area
params.filterByArea = True
params.minArea = 150
params.maxArea = 80000

# Filter by Circularity
params.filterByCircularity = False
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

#TypeError: 'bool' object is not subscriptable

def get_color_blobs(img_BGR, rangeMin=(160, 80, 50), rangeMax=(10, 255, 255), plot_result=False):
    # keypoints on original image (will look for blobs in grayscale)
    image = cv2.cvtColor(img_BGR,cv2.COLOR_BGR2HSV)

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

    # apply the mask
    keypoints = detector.detect(255-mask)
    #keypoints = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1, 150, param1=100, param2=20, minRadius=20, maxRadius=200)


    if plot_result:
        regions = cv2.bitwise_and(img_BGR, img_BGR, mask = mask)
        regions = cv2.cvtColor(regions,cv2.COLOR_HSV2BGR)
        cv2.imshow("Detected Regions", np.hstack([img_BGR, regions]))

        for kp in keypoints:
            print(kp.pt[0], kp.pt[1], kp.size)

        # Show mask and blobs found
        im_with_keypoints = cv2.drawKeypoints(img_BGR, keypoints, np.array([]),
            (255,255,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        cv2.imshow("Keypoints on RED", im_with_keypoints)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return keypoints


def get_blob(img_BGR, rangeMin=(160, 80, 50), rangeMax=(10, 255, 255)):
    blobs = get_color_blobs(img_BGR, rangeMin, rangeMax, False)
    blob = max(blobs, default=None, key= lambda item: item.size)
    
    return (blob.pt[0] / np.size(img_BGR,0), blob.pt[1] / np.size(img_BGR,1))
