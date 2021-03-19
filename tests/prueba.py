from functions.get_color_blobs import get_color_blobs, get_blob
import cv2
import numpy as np
import time

ESC = 27
KNN = 1
MOG2 = 2

#rawCapture = cv2.imread("./media.png")
#rawCapture = cv2.imread("./AM-lejos.png")
rawCapture = cv2.imread("./lejos.png")

get_color_blobs(rawCapture, plot_result=True)

kp = get_blob(rawCapture)

print(kp[0], kp[1])
