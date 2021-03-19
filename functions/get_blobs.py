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


def get_color_blobs(img_BGR,plot_result=True):
    # keypoints on original image (will look for blobs in grayscale)
    image = cv2.cvtColor(img_BGR,cv2.COLOR_BGR2HSV)

    # HSV FORMAT ranges
    redMin1 = (0, 100, 100)
    redMax1 = (10, 255, 255)

    redMin2 = (350, 100, 100)
    redMax2 = (360, 255, 255)

    # Obtain the mask
    mask1 = cv2.inRange(image, redMin1, redMax1)
    mask2 = cv2.inRange(image, redMin2, redMax2)
    mask = mask1 | mask2

    # some morphological operations (closing) to remove small blobs
    #mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
    #mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (8, 8)))

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

if __name__ == '__main__':
    # Read image
    img_BGR = cv2.imread("../tests/red_blue.jpg")
    # img_BGR = cv2.imread("tests/many.jpg")
    get_color_blobs(img_BGR)
