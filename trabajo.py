import time

import cv2
import matplotlib
import numpy as np

from classes import Cfg
from classes.Map import GRID
from classes.MapLib import Map2D
from classes.Periodic import Periodic
from classes.Robot import Robot
from p4 import traverseLabyrinthFine

matplotlib.use("TkAgg")  # sudo apt-get install tcl-dev tk-dev python-tk python3-tk if TkAgg is not available

Cfg.add_argument("-bb8", "--bb8", help="detect bb8", action="store_true")
Cfg.add_argument("-arc", "--S_as_arcs", help="do s as arcs", action="store_true")

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

        # press button to start
        robot.waitButtonPress()

        # start odometry
        robot.startOdometry()

        # detect color
        leftSide = robot.getLight() <= 0.5
        if(leftSide): print("leftSide")
        if(not leftSide): print("rightSide")
        enter, exit = (2, 0) if leftSide else (0, 2)

        # perform S
        if Cfg.S_as_arcs:
            # as arcs
            T = 7
            for side, c in ((1 if leftSide else -1, 6), (-1 if leftSide else 1, 4)):
                cx, cy = myMap._cell2pos(1, c)
                for t in range(T):
                    angle = np.pi * t / (T - 1)
                    robot.go(cx - side * GRID * np.sin(angle), cy + GRID * np.cos(angle))
        else:
            # as squares
            # robot.go(*myMap._cell2pos(enter, 7))
            # robot.go(*myMap._cell2pos(enter, 6))
            # robot.go(*myMap._cell2pos(enter, 5))
            # robot.go(*myMap._cell2pos(1, 5))
            # robot.go(*myMap._cell2pos(exit, 5))
            # robot.go(*myMap._cell2pos(exit, 4))
            # robot.go(*myMap._cell2pos(exit, 3))
            # robot.go(*myMap._cell2pos(1, 3))
            """
            robot.rotate(-np.pi / 2)
            robot.advance(GRID)
            robot.rotate(np.pi / 2)
            robot.advance(2 * GRID)
            robot.rotate(np.pi / 2)
            robot.advance(2 * GRID)
            robot.rotate(-np.pi / 2)
            robot.advance(2 * GRID)
            robot.rotate(-np.pi / 2)
            robot.advance(GRID)
            """

        # enter labyrinth
        #robot.go(*myMap._cell2pos(enter, 3))
        #robot.go(*myMap._cell2pos(enter, 2))
        robot.advance(GRID)
        robot.rotate(np.pi / 2)
        robot.advance(GRID)

        # traverse labyrinth
        robot.onMarker(x=GRID + Cfg.LIGHT_OFFSET * (1 if leftSide else -1))
        #traverseLabyrinth((exit, 2), myMap, robot)
        traverseLabyrinthFine((enter, 0), (exit, 0), 2, myMap, robot)

        exit()

        # exit labyrinth
        robot.onMarker(y=GRID * 3 + Cfg.LIGHT_OFFSET)
        robot.go(*myMap._cell2pos(exit, 3))
        robot.onMarker()
        robot.go(*myMap._cell2pos(1, 3.5))

        # look for ball
        robot.trackObject()

        # position looking at the images
        robot.go(*myMap._cell2pos(0.5, 6))
        robot.lookAt(*myMap._cell2pos(0.5, 7))

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
                robot.setSpeed(0, np.deg2rad(10) * rotateLeft)
                rotateLeft = 1 - rotateLeft

        # exit lab
        if leftExit:
            robot.go(*myMap._cell2pos(0, 7))
            robot.go(*myMap._cell2pos(-1, 7))
        else:
            robot.go(*myMap._cell2pos(2, 7))
            robot.go(*myMap._cell2pos(2, 8))

        time.sleep(3)

    finally:
        # wrap up and close stuff before exiting
        if robot is not None: robot.stopOdometry()
