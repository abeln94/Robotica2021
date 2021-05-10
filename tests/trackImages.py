import time

import cv2
import numpy as np

from classes import Cfg
from classes.Map import GRID
from classes.Periodic import Periodic
from classes.Robot import Robot

Cfg.add_argument("-bb8", "--bb8", help="detect bb8", action="store_true")
robot = None


#
def trackImages(our, other):
    MOVEMENT_TIME = 0.1  # seconds
    ANGULAR_SPEED_LOST = np.deg2rad(30)  # angular speed when no image found
    BACKTRACK_VELOCITY = 25  # mm/s

    distance = robot.getObstacleDistance()

    # 1. Loop running the tracking until target (centroid position and distance) reached
    periodic = Periodic()
    while periodic(abs(distance - GRID) > 0.1):

        if distance > GRID:

            # 1.1. search the most promising blob ..
            foundOther, coordinatesOther = robot.detectImage(other)
            foundOur, coordinatesOur = robot.detectImage(our)

            # 1.2. check the given position
            if foundOur and foundOther:
                # 1.3 blob found, check its position for planning movement
                xOur = coordinatesOur[0]
                yOur = coordinatesOur[1]
                xOther = coordinatesOther[0]
                yOther = coordinatesOther[1]

                print("found out at (x=", xOur, "y=", yOur, ")")
                print("found other at (x=", xOther, "y=", yOther, ")")

                dif = abs(xOther - xOur)

                if 0.45 < dif < 0.55:
                    # 1.4 bad orientation
                    robot.setSpeed(Cfg.LIN_VEL, dif * Cfg.ANG_VEL)
                else:
                    # 1.5 good orientation, just advance
                    robot.setSpeed(Cfg.LIN_VEL, 0)

            else:
                # 1.3 no found
                robot.setSpeed(0, ANGULAR_SPEED_LOST)

        else:
            robot.setSpeed(-BACKTRACK_VELOCITY, 0)

        time.sleep(MOVEMENT_TIME)
        distance = robot.getObstacleDistance()

    # Returns whether our is at the lefth position or not
    return coordinatesOur[0] > coordinatesOther[0]


if __name__ == "__main__":
    try:

        # Initialize Odometry. Default value will be 0,0,0
        robot = Robot()
        # start odometry
        robot.startOdometry()
        time.sleep(5)

        # 1. 
        image_R2D2 = cv2.flip(cv2.imread("images/R2-D2_s.png", cv2.IMREAD_COLOR), -1)
        image_BB8 = cv2.flip(cv2.imread("images/BB8_s.png", cv2.IMREAD_COLOR), -1)

        our, other = (image_R2D2, image_BB8) if not Cfg.bb8 else (image_BB8, image_R2D2)

        # 2. 
        print("Press ESC to quit")
        print(trackImages(our, other))

    finally:
        # wrap up and close stuff before exiting
        if robot is not None: robot.stopOdometry()
