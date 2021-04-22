import cv2

from classes import Cfg
from classes.Robot import Robot

Cfg.add_argument("-bb8", "--bb8", help="detect bb8", action="store_true")

robot = None

#

if __name__ == "__main__":
    try:

        # Initialize Odometry. Default value will be 0,0,0
        robot = Robot()

        # 1. 
        image_R2D2 = cv2.imread("images/R2-D2_s.png", cv2.IMREAD_COLOR)
        image_BB8 = cv2.imread("images/BB8_s.png", cv2.IMREAD_COLOR)

        our, other = (image_R2D2, image_BB8) if not Cfg.bb8 else (image_BB8, image_R2D2)

        # 2. 
        print("Press ESC to quit")
        while cv2.waitKey(1000) != 27:
            foundOur, coordinatesOur = robot.detectImage(our)
            foundOther, coordinatesOther = robot.detectImage(other)

            if foundOur and foundOther:
                if coordinatesOur[0] > coordinatesOther[0]:  # the camera is inverted
                    print('Our is at the left')
                else:
                    print('Our is at the right')
            else:
                print('not found', foundOur, foundOther)


    finally:
        # wrap up and close stuff before exiting
        if robot is not None: robot.stopOdometry()
