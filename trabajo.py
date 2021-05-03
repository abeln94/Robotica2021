import time

import cv2
import matplotlib
import numpy as np

from classes import Cfg
from classes.Map import GRID
from classes.MapLib import Map2D
from classes.Periodic import Periodic
from classes.Robot import Robot
from p4 import traverseLabyrinthFine, traverseLabyrinth

matplotlib.use("TkAgg")  # sudo apt-get install tcl-dev tk-dev python-tk python3-tk if TkAgg is not available

Cfg.add_argument("-bb8", "--bb8", help="detect bb8", action="store_true")
Cfg.add_argument("-skip", help="start at the end of the labyrinth", action="store_true")
Cfg.add_argument("-odo", help="Use the odometry for movement", action="store_true")

# images
IMAGE_R2D2 = cv2.flip(cv2.imread("images/R2-D2_s.png", cv2.IMREAD_COLOR), -1)
IMAGE_BB8 = cv2.flip(cv2.imread("images/BB8_s.png", cv2.IMREAD_COLOR), -1)
IMAGE_OUR, IMAGE_OTHER = (IMAGE_R2D2, IMAGE_BB8) if not Cfg.bb8 else (IMAGE_BB8, IMAGE_R2D2)

robot = None

if __name__ == "__main__":
    try:

        # load map
        myMap = Map2D(Cfg.FOLDER_MAPS + 'mapa0.txt')
        myMap.sizeCell = GRID  # hardcoded because it should not be in the file!!!!

        # init Robot
        robot = Robot([*myMap._cell2pos(1, 7), np.deg2rad(-90)])


        def go(x, y):
            robot.go(*myMap._cell2pos(x, y))


        def lookAt(x, y):
            robot.lookAt(*myMap._cell2pos(x, y))


        # press button to start
        robot.waitButtonPress()

        # start odometry
        robot.startOdometry()

        # detect color
        leftSide = robot.getLight() <= 0.5
        sideMul = 1 if leftSide else -1
        enter, exit = (2, 0) if leftSide else (0, 2)

        if not Cfg.skip:

            # perform S as squares
            if Cfg.odo:
                go(enter, 7)
                go(enter, 6)
                go(enter, 5)
                go(1, 5)
                go(exit, 5)
                go(exit, 4)
                go(exit, 3)
                go(1, 3)
            else:
                robot.rotate(np.pi / 2 * sideMul)
                robot.advance(GRID)
                robot.rotate(-np.pi / 2 * sideMul)
                robot.advance(2 * GRID)
                robot.rotate(-np.pi / 2 * sideMul)
                robot.advance(2 * GRID)
                robot.rotate(np.pi / 2 * sideMul)
                robot.advance(2 * GRID)
                robot.rotate(np.pi / 2 * sideMul)
                robot.advance(GRID)

            # enter labyrinth
            if Cfg.odo:
                go(enter, 3)
                go(enter, 2)
            else:
                robot.advance(GRID)
                robot.rotate(-np.pi / 2 * sideMul)
                robot.advance(GRID)

            # traverse labyrinth
            robot.onMarker(x=GRID + Cfg.LIGHT_OFFSET * (-1 if leftSide else 1))
            if Cfg.odo:
                traverseLabyrinth((exit, 2), myMap, robot)
            else:
                traverseLabyrinthFine((0, enter), (0, exit), 2, myMap, robot)

        # exit labyrinth
        robot.onMarker(x=GRID * (exit + 0.5), y=GRID * 3 + Cfg.LIGHT_OFFSET, th=np.deg2rad(90))
        if Cfg.odo:
            robot.go(*myMap._cell2pos(exit, 3))
        else:
            robot.advance(GRID)
        robot.onMarker()

        # look for ball
        if Cfg.odo:
            go(1, 3)
            lookAt(1, 4)
        else:
            robot.rotate(-np.pi / 2 * sideMul)
            robot.advance(GRID)
            robot.rotate(np.pi / 2 * sideMul)
        robot.trackObject()

        # # recolocate odometry
        # _, _, th = robot.readOdometry()
        # robot.rotate(norm_pi(np.deg2rad(180) - th))
        # robot.advance(robot.getObstacleDistance() - GRID)
        # dist = robot.updateOdOnWall()
        # robot.onMarker(x=dist, th=np.deg2rad(180), now=True)
        #
        # robot.advance(dist - GRID)
        # robot.rotate(np.deg2rad(-90))
        #
        # robot.advance(robot.getObstacleDistance() - GRID * 1.5)
        # dist = robot.updateOdOnWall(ANG=30)
        #
        # robot.onMarker(y=GRID * 8 - dist, th=np.deg2rad(90), now=True)

        # position looking at the images
        go(0.5, 6)
        lookAt(0.5, 7)

        # detect image
        periodic = Periodic(1)
        rotateLeft = 1
        while periodic():
            foundOur, coordinatesOur = robot.detectImage(IMAGE_OUR)
            foundOther, coordinatesOther = robot.detectImage(IMAGE_OTHER)
            if foundOur and foundOther:
                leftExit = coordinatesOur[0] > coordinatesOther[0]  # the camera is inverted
                break
            else:
                robot.setSpeed(0, np.deg2rad(7.5) * rotateLeft)
                rotateLeft = 1 - rotateLeft

        # dist = robot.updateOdOnWall(30)
        # robot.advance(dist - GRID * 0.5)

        go(0.5, 6.75)
        dist = robot.updateOdOnWall(45)
        if dist > GRID / 2:
            robot.advance(dist - GRID / 2)

        robot.rotate(np.deg2rad(-90))

        dist = robot.getObstacleDistance()
        if dist > GRID / 2:
            robot.advance(dist - GRID / 2)

        robot.updateOdOnWall(45)

        if leftExit:
            robot.rotate(np.deg2rad(-180))
            robot.advance(3 * GRID)
        else:
            robot.rotate(np.deg2rad(90))
            robot.advance(GRID)

        time.sleep(3)

    finally:
        # wrap up and close stuff before exiting
        if robot is not None: robot.stopOdometry()
