import cv2
import time
import sys
from classes import Cfg
from classes.Robot import Robot

robot = None

#

if __name__ == "__main__":
    try:

        # Initialize Odometry. Default value will be 0,0,0
        robot = Robot()

        # 1. 
        image = cv2.imread("images/R2-D2_s.png", cv2.IMREAD_COLOR)

        # 2. 
        print("Press ESC to quit")
        stop = False
        while not stop:
            found = robot.detectImage(image)
            print("Returned (", found, ", ", None, ")")
            if found: # Wait for press key
                cv2.waitKey(0)
            time.sleep(1)
            if cv2.waitKey(1) == 27:
                stop = True

    finally:
        # wrap up and close stuff before exiting
        if robot is not None: robot.stopOdometry()
