from functions.get_blobs import get_color_blobs
import cv2
import numpy as np
import time

ESC = 27
KNN = 1
MOG2 = 2

rawCapture = cv2.imread("./pintado.png")

key_points = get_color_blobs(rawCapture)

for kp in key_points:
    print(kp.pt[0], kp.pt[1], kp.size)
