import time

import picamera
from functions.get_blobs import get_color_blobs
from picamera.array import PiRGBArray

ESC = 27
KNN = 1
MOG2 = 2

cam = picamera.PiCamera()

cam.resolution = (320, 240)
# cam.resolution = (640, 480)
cam.framerate = 32
rawCapture = PiRGBArray(cam, size=(320, 240))
# rawCapture = PiRGBArray(cam, size=(640, 480))

# allow the camera to warmup
time.sleep(0.1)

cam.capture(rawCapture, format="bgr")

key_points = get_color_blobs(rawCapture.array)

# cv2.imwrite('./foto.png', rawCapture)
