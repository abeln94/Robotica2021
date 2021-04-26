import cv2

from classes import Cfg
from classes.Robot import Robot
from functions.get_image_match import match_images

Cfg.add_argument("-bb8", "--bb8", help="detect bb8", action="store_true")
Cfg.add_argument("--use_image", help="use the given image instead of capturing picamera", default=None)
robot = None

#

if __name__ == "__main__":
    try:

        # Initialize Odometry. Default value will be 0,0,0
        robot = Robot()

        # 1. 
        image_R2D2 = cv2.flip(cv2.imread("images/R2-D2_s.png", cv2.IMREAD_COLOR), -1)
        image_BB8 = cv2.flip(cv2.imread("images/BB8_s.png", cv2.IMREAD_COLOR), -1)

        our, other = (image_R2D2, image_BB8) if not Cfg.bb8 else (image_BB8, image_R2D2)

        capture = None if not Cfg.use_image else cv2.flip(cv2.imread(Cfg.use_image, cv2.IMREAD_COLOR), -1)

        # 2. 
        print("Press ESC to quit")
        while cv2.waitKey(1000) != 27:
            if not Cfg.use_image:
                foundOther, coordinatesOther = robot.detectImage(other)
                foundOur, coordinatesOur = robot.detectImage(our)
            else:
                foundOther, coordinatesOther = match_images(other, capture)
                foundOur, coordinatesOur = match_images(our, capture)
            
            if foundOur and foundOther:
                if coordinatesOur[0] < coordinatesOther[0]:  # the camera is inverted
                    print('Our is at the left')
                else:
                    print('Our is at the right')
            else:
                print('not found', foundOur, foundOther)


    finally:
        # wrap up and close stuff before exiting
        if robot is not None: robot.stopOdometry()
