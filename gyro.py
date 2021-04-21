import random
import time

import matplotlib
import numpy as np

from classes import Cfg
from classes.Map import GRID
from classes.MapLib import Map2D
from classes.Robot import Robot
from p4 import traverseLabyrinth

matplotlib.use("TkAgg")  # sudo apt-get install tcl-dev tk-dev python-tk python3-tk if TkAgg is not available

robot = None

if __name__ == "__main__":
    try:

        # load map
        myMap = Map2D(Cfg.FOLDER_MAPS + 'mapa0.txt')
        myMap.sizeCell = GRID  # hardcoded because it should not be in the file!!!!

        # init Robot
        robot = Robot([*myMap._cell2pos(1, 7), np.deg2rad(-90)])
        robot.startOdometry()

        # press button to start
        robot.waitButtonPress()

        # spin and wait
        robot.setSpeed(0, np.deg2rad(45))
        time.sleep(10)

    finally:
        # wrap up and close stuff before exiting
        if robot is not None: robot.stopOdometry()
