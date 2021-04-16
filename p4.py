import os
import time

import matplotlib

from classes import Cfg
from classes.Map import GRID
from classes.MapLib import Map2D
from classes.Robot import Robot
from functions.functions import norm_pi

matplotlib.use("TkAgg")  # sudo apt-get install tcl-dev tk-dev python-tk python3-tk if TkAgg is not available

# args
Cfg.add_argument("-m", "--mapfile", help="path to find map file", default="mapa1.txt")
Cfg.add_argument("-start", "--startCell", help="Cell where the robot starts", default=(0, 0), nargs=2, type=int)
Cfg.add_argument("-end", "--endCell", help="Cell where the robot ends", default=(-1, -1), nargs=2, type=int)

robot = None

if __name__ == "__main__":
    try:

        # check map file
        mapFile = Cfg.FOLDER_MAPS + Cfg.mapfile
        os.makedirs(os.path.dirname(mapFile), exist_ok=True)
        if not os.path.isfile(mapFile):
            print('Map file %s does not exist' % Cfg.mapfile)
            exit(1)

        # 1. load map
        myMap = Map2D(mapFile)
        myMap.sizeCell = GRID  # hardcoded because it should not be in the file!!!!
        initial_position = [Cfg.startCell[0] % myMap.sizeX, Cfg.startCell[1] % myMap.sizeY]
        target_position = [Cfg.endCell[0] % myMap.sizeX, Cfg.endCell[1] % myMap.sizeY]

        # 2. init Robot and launch updateOdometry thread()
        robot = Robot([*myMap._cell2pos(*initial_position), 0])
        robot.startOdometry()

        time.sleep(3)

        # 3. perform trajectory
        # get path
        path = myMap.planPath(*initial_position, *target_position)
        current_index = 0
        while current_index < len(path) - 1:
            # while not at the last cell, go to the next
            next_pos = myMap._cell2pos(*(path[current_index + 1]))

            if robot.detectObstacle(*next_pos):
                # there is an obstacle, can't go there directly

                # delete the 3 front connections (a wall extends to the corners too)
                current_cell = path[current_index]
                neighbour = myMap.getNeighbour(norm_pi(robot.th.value))
                myMap.deleteConnection(*current_cell, (neighbour - 1) % 8)
                myMap.deleteConnection(*current_cell, neighbour)
                myMap.deleteConnection(*current_cell, (neighbour + 1) % 8)

                # get the new path
                path = myMap.planPath(*current_cell, *target_position)
                current_index = 0
            else:
                # no obstacle, go
                robot.go(*next_pos)
                current_index += 1

        robot.setSpeed(0, 0)
        myMap.drawMapWithRobotLocations([robot.readOdometry()])

    finally:
        # wrap up and close stuff before exiting
        if robot is not None: robot.stopOdometry()
