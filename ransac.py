import cv2

from classes.Periodic import Periodic
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
        periodic = Periodic(1)
        while periodic(cv2.waitKey(1) != 27):
            found, coordinates = robot.detectImage(image)
            print("Returned (", found, ", ", coordinates, ")")

    finally:
        # wrap up and close stuff before exiting
        if robot is not None: robot.stopOdometry()
