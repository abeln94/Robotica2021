import os

import matplotlib
import numpy as np

from classes import Cfg
from classes.Map import GRID
from classes.MapLib import Map2D
from classes.Robot import Robot
from functions.functions import norm_pi

matplotlib.use("TkAgg")  # sudo apt-get install tcl-dev tk-dev python-tk python3-tk if TkAgg is not available

Cfg.add_argument("-m", "--mapfile", help="path to find map file", default="mapa1.txt")

# NOTES ABOUT TASKS to DO in P4:
# 1)findPath(x1,y1, x2,y2),   fillCostMatrix(), replanPath () --> should be methods from the new Map2D class
# 2) go(x,y) and detectObstacle() could be part of your Robot class (depending how you have implemented things)
# 3) you can change these method signatures if you need, depending how you have implemented things

robot = None

if __name__ == "__main__":
    try:

        # check map file
        mapFile = Cfg.FOLDER_MAPS + Cfg.mapfile
        os.makedirs(os.path.dirname(mapFile), exist_ok=True)
        if not os.path.isfile(mapFile):
            print('Map file %s does not exist' % Cfg.mapfile)
            exit(1)

        # Instantiate Odometry with your own files from P2/P3
        robot = Robot([GRID / 2, GRID / 2, np.deg2rad(90)])

        # 1. load map and compute costs and path
        myMap = Map2D(mapFile)
        myMap.sizeCell = GRID  # hardcoded because it should not be in the file!!!!
        target_position = [2, 0]  # CHANGE ME
        # myMap.verbose = True
        # myMap.drawMap(saveSnapshot=False)

        # you can set verbose to False to stop displaying plots interactively
        # (and maybe just save the snapshots of the map)
        # myMap.verbose = False

        # sample commands to see how to draw the map
        # sampleRobotLocations = [[0, 0, 0], [600, 600, 3.14]]
        # this will save a .png with the current map visualization,
        # all robot positions, last one in green
        # myMap.verbose = True
        # myMap.drawMapWithRobotLocations(sampleRobotLocations, saveSnapshot=False)

        # this shows the current, and empty, map and an additionally closed connection
        # myMap.deleteConnection(0, 0, 0)
        # myMap.verbose = True
        # myMap.drawMap(saveSnapshot=False)

        # this will open a window with the results, but does not work well remotely
        # myMap.verbose = True
        # sampleRobotLocations = [[200, 200, 3.14 / 2.0], [200, 600, 3.14 / 4.0], [200, 1000, -3.14 / 2.0], ]
        # myMap.drawMapWithRobotLocations(sampleRobotLocations, saveSnapshot=False)
        # myMap.closeAll()

        # 2. launch updateOdometry thread()
        robot.startOdometry()

        # 3. perform trajectory
        x, y, th = robot.readOdometry()
        initial_position = myMap._pos2cell(x, y)
        path = myMap.planPath(*initial_position, *target_position)
        current_index = 0
        while current_index < len(path) - 1:

            # check if there is an obstacle
            destiny = myMap._cell2pos(*(path[current_index + 1]))
            robot.lookAt(*destiny)

            # if there is, replan
            if robot.getObstacleDistance() < myMap.sizeCell:
                # there is an obstacle
                th = norm_pi(robot.th.value)
                vecino = ((th + np.pi / 8) // (np.pi / 4) + 2) % 8  # TODO: extract function

                current_pos = path[current_index]
                myMap.deleteConnection(*current_pos, (vecino - 1) % 8)
                myMap.deleteConnection(*current_pos, vecino)
                myMap.deleteConnection(*current_pos, (vecino + 1) % 8)

                path = myMap.planPath(*current_pos, *target_position)
                current_index = 0
            else:
                # no obstacle, go
                robot.go(*destiny)
                current_index += 1

        #
        # check if there are close obstacles
        # deal with them...
        # Avoid_obstacle(...) OR RePlanPath(...)

    finally:
        # wrap up and close stuff before exiting
        if robot is not None: robot.stopOdometry()
