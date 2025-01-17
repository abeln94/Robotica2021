#!/usr/bin/python
# -*- coding: UTF-8 -*-
from __future__ import division  # ''
from __future__ import print_function  # use python 3 syntax but make it compatible with python 2

import os

import matplotlib.pyplot as plt
import numpy as np

from classes import Cfg

Cfg.add_argument("-pm", "--plotMap", help="Plot internal map weights", action="store_true")
Cfg.add_argument("-fm", "--logMap", help="Log map into a file", default=False)


class Map2D:
    def __init__(self, map_description_file):
        """
        Load and initialize map from file. \

        map_description_file: path to a text file containing map description in the standard format. \
        Example for a 3x3 grid map, with (squared) cells of 400mm side length called mapa0. \
        All free space, i.e., all connections between cells are open, except those on the limits of the map.
        For more details on the format, see class documentation.

        mapa0.txt content:
        3 3 400
        0 0 0 0 0 0 0
        0 1 1 1 1 1 0
        0 1 1 1 1 1 0
        0 1 1 1 1 1 0
        0 1 1 1 1 1 0
        0 1 1 1 1 1 0
        0 0 0 0 0 0 0

        """
        # params to visualize
        self.mapLineStyle = 'r-'
        self.costValueStyle = 'g*'
        self.current_ax = None

        # variables about map params
        self.sizeX = 0
        self.sizeY = 0
        self.sizeCell = 0

        self.connectionMatrix = None
        self.costMatrix = None
        self.currentPath = None

        if self._loadMap(map_description_file):
            print("Map %s loaded ok" % map_description_file)
        else:
            print("Map %s NOT loaded" % map_description_file)

        if Cfg.plotMap:
            # create a new figure and set it as current axis
            plt.figure('map')
            plt.plot([], [])
            plt.show(block=False)
            self.drawMap()

    # from python docs: https://docs.python.org/3/tutorial/classes.html#private-variables
    # “Private” instance variables that cannot be accessed except from inside an object don’t exist in Python.
    # However, there is a convention that is followed by most Python code: a name prefixed with an underscore \
    # (e.g. _spam) should be treated as a non-public part of the API (whether it is a function, a method or a data member).

    # ############################################################
    # private methods
    # ############################################################
    def _initConnections(self, init_value=0):
        """
        to initialize the matrix, we set all connections to be closed.
        When the file with the description is loaded, it will "open" (set to 1) the corresponding ones.
        """
        self.connectionMatrix = np.ones((2 * self.sizeX + 1, 2 * self.sizeY + 1)) * init_value

    def _initCostMatrix(self, init_value=-2):
        """
        to initialize the matrix, we set all connections to be closed.
        When the file with the description is loaded, it will "open" (set to 1) the corresponding ones.
        """
        self.costMatrix = np.ones((self.sizeX, self.sizeY)) * init_value

        # Example costMatrix (filled manually!) for Map1
        # if we plan to go from 0,0 to 2,0
        # self.costMatrix[2,0] = 0
        # self.costMatrix[1,0] = 1
        # self.costMatrix[1,1] = 2
        # self.costMatrix[1,2] = 3
        # self.costMatrix[0,2] = 4
        # self.costMatrix[2,2] = 4
        # self.costMatrix[0,1] = 5
        # self.costMatrix[2,1] = 5
        # self.costMatrix[0,0] = 6

    def _loadMap(self, mapFileName):
        """
        Load map from a txt file (mapFileName) to fill the map params and connectionMatrix. \
        NOTES: \
        \t connectionMatrix is a numpy array \
        \t Function will return False if something went wrong loading the map file.
        """
        verbose = True
        try:
            # FILL GLOBAL VARIABLES dimX dimY cellSize
            loadingOk = False
            mapF = open(mapFileName, "r")

            # 1. special case for first line. initialize dimX dimY cellSize
            header = mapF.readline()  # next()
            tmp = header.split()  # any whitespace string is a separator and empty strings are removed from the result
            if verbose:
                print("Header line: %s " % header)
            parsed_header = [int(c) for c in tmp]
            # expected to have three numbers: sizeX sizeY sizeCell_in_mm
            if len(parsed_header) == 3:
                self.sizeX, self.sizeY, self.sizeCell = parsed_header
            else:
                print("Wrong header in map file: %s" % header)
                return False

            # 2.init connectionMatrix and costMatrix
            self._initConnections()
            self._initCostMatrix()

            # 3. load rest of the map connection lines information
            for indx, line in enumerate(mapF):
                # we start loading from the file the "top" row of the map
                current_row = (self.connectionMatrix.shape[1] - 1) - indx
                # Split numbers in the line. Any whitespace string is a separator and empty strings are removed from the result
                tmp = line.split()
                if verbose:
                    print("Line for map row %d: %s " % (current_row, line))
                parsed_line = [int(c) for c in tmp]

                if len(parsed_line) == self.connectionMatrix.shape[0] and indx < self.connectionMatrix.shape[1]:
                    self.connectionMatrix[:, current_row] = parsed_line
                elif len(parsed_line):  # don't give errors because of empty lines
                    print("Wrong connectionMatrix (%s) row data: %s" % (self.connectionMatrix.shape(), line))
                    return False
            mapF.close()
            loadingOk = True
        except Exception as e:
            print("ERROR:", e.__doc__)
            print(e)
            # raise
            loadingOk = False

        return loadingOk

    def _cell2connCoord(self, cellX, cellY, numNeigh):
        """
        Input:
            cellX, cellY: cell coordinates (cellX, cellY) in the map grid
            numNeigh: index of one of the cell 8-neighbours

        Output:
            (connX,connY): 2D coordinates (in the connectionMatrix!!) \
            of the connection of the input cell to the input neighbour
        """
        connX = 2 * cellX + 1
        connY = 2 * cellY + 1

        return self._neighbour(connX, connY, numNeigh)

    def _neighbour(self, cellX, cellY, numNeigh):
        """
        Returns the cell which corresponds to numNeigh
        """
        p = [cellX, cellY]

        result = {
            0: lambda p: [p[0], p[1] + 1],
            1: lambda p: [p[0] + 1, p[1] + 1],
            2: lambda p: [p[0] + 1, p[1]],
            3: lambda p: [p[0] + 1, p[1] - 1],
            4: lambda p: [p[0], p[1] - 1],
            5: lambda p: [p[0] - 1, p[1] - 1],
            6: lambda p: [p[0] - 1, p[1]],
            7: lambda p: [p[0] - 1, p[1] + 1],
        }

        return result[numNeigh](p)

    def _cell(self, thisCell, destCell):
        """
        Returns the neighbour which corresponds to destCell
        TODO: make this more efficient (something similar to _neighbour)
        """
        for i in range(0, 8):
            if self._neighbour(*thisCell, i) == destCell:
                return i  # relation encountered
        return -1  # They arent neighbours

    def _pos2cell(self, x_mm, y_mm):
        """ Convert from robot odometry coordinates (in mm) to cell coordinates """
        # make sure we discretize the result to the closest lower integer value
        x_cell = int(np.floor(x_mm / self.sizeCell))
        y_cell = int(np.floor(y_mm / self.sizeCell))
        return [x_cell, y_cell]

    def _cell2pos(self, x_pos, y_pos):
        """ Convert from cell coordinates to robot coordinates (in mm) """
        return [(x + 0.5) * self.sizeCell for x in [x_pos, y_pos]]

    # ############################################################
    # public methods
    # ############################################################
    def setConnection(self, cellX, cellY, numNeigh):
        """
        open a connection, i.e., we can go straight from cellX,cellY to its neighbour number numNeigh
        """
        # from coordinates in the grid of cells to coordinates in the connection matrix
        [connX, connY] = self._cell2connCoord(cellX, cellY, numNeigh)
        self.connectionMatrix[connX, connY] = 1  # True

    def deleteConnection(self, cellX, cellY, numNeigh):
        """
        close a connection, i.e., we can NOT go straight from cellX,cellY to its neighbour number numNeigh
        """
        # from coordinates in the grid of cells to coordinates in the connection matrix
        [connX, connY] = self._cell2connCoord(cellX, cellY, numNeigh)
        self.connectionMatrix[connX, connY] = 0  # False

    def isConnected(self, cellX, cellY, numNeigh):
        """
        returns True if the connnection from cell (x,y) to its neighbour number numNeigh is open.

        The neighbour indexing is considered as follows
        (8-neighbours from cell x,y numbered clock-wise):

        7     0       1
        6   (x,y)     2
        5     4       3

        """
        [connX, connY] = self._cell2connCoord(cellX, cellY, numNeigh)
        return self.connectionMatrix[connX, connY]

    # aux functions to display (or save image) with robot and map stuff
    def _drawGrid(self):
        """
        aux function to create a grid with map lines
        """
        if not self.current_ax:
            print("Error plotting: do not call this function directly, \
                call drawMap first to create a plot where to draw")
            return False

        plt.rc('grid', linestyle="--", color='gray')
        plt.grid(True)
        plt.tight_layout()

        x_t = range(0, (self.sizeX + 1) * self.sizeCell, self.sizeCell)
        y_t = range(0, (self.sizeY + 1) * self.sizeCell, self.sizeCell)
        x_labels = [str(n) for n in x_t]
        y_labels = [str(n) for n in y_t]
        plt.xticks(x_t, x_labels)
        plt.yticks(y_t, y_labels)

        # Main rectangle
        X = np.array([0, self.sizeX, self.sizeX, 0, 0]) * self.sizeCell
        Y = np.array([0, 0, self.sizeY, self.sizeY, 0]) * self.sizeCell
        self.current_ax.plot(X, Y, self.mapLineStyle)

        # "vertical" walls
        for i in range(2, 2 * self.sizeX, 2):
            for j in range(1, 2 * self.sizeY, 2):
                if not self.connectionMatrix[i, j]:
                    # paint "right" wall from cell (i-1)/2, (j-1)/2
                    cx = np.floor((i - 1) / 2)
                    cy = np.floor((j - 1) / 2)
                    X = np.array([cx + 1, cx + 1]) * self.sizeCell
                    Y = np.array([cy, cy + 1]) * self.sizeCell
                    self.current_ax.plot(X, Y, self.mapLineStyle)

        # "horizontal" walls
        for j in range(2, 2 * self.sizeY, 2):
            for i in range(1, 2 * self.sizeX, 2):
                if not self.connectionMatrix[i, j]:
                    # paint "top" wall from cell (i-1)/2, (j-1)/2
                    cx = np.floor((i - 1) / 2)
                    cy = np.floor((j - 1) / 2)
                    X = np.array([cx, cx + 1]) * self.sizeCell
                    Y = np.array([cy + 1, cy + 1]) * self.sizeCell
                    self.current_ax.plot(X, Y, self.mapLineStyle)
        plt.axis('equal')

        return True

    # aux functions to display the current CostMatrix on the map
    def _drawCostMatrix(self):
        """
        aux function to create a grid with map lines
        """
        if not self.current_ax:
            print("Error plotting: do not call this function directly, \
                call drawMap first to create a plot where to draw")
            return False

        # "center" of each cell
        for i in range(0, self.sizeX):
            for j in range(0, self.sizeY):
                cx = i * self.sizeCell + self.sizeCell / 2.
                cy = j * self.sizeCell + self.sizeCell / 2.
                X = np.array([cx])
                Y = np.array([cy])
                cost = self.costMatrix[i, j]
                self.current_ax.text(X, Y, str(cost))

        plt.axis('equal')

        return True

    # Dibuja robot en location_eje con color (c) y tamano (p/g)
    def _drawRobot(self, loc_x_y_th=[0, 0, 0], robotPlotStyle='b', small=True):
        """
        UPDATES existing plot to include current robot position
        It expects an existing open figure (probably with the map already on it)

        loc_x_y_th is the position x,y and orientation in mm and radians of the main axis of the robot

        """
        if not self.current_ax:
            print("Error plotting: do not call this function directly, \
                call drawMap first to create a plot where to draw")
            return False

        if small:
            largo, corto, descentre = [80, 50, 5]
        else:
            largo, corto, descentre = [160, 100, 10]

        trasera_dcha = np.array([-largo, -corto, 1])
        trasera_izda = np.array([-largo, corto, 1])
        delantera_dcha = np.array([largo, -corto, 1])
        delantera_izda = np.array([largo, corto, 1])
        frontal_robot = np.array([largo, 0, 1])

        tita = loc_x_y_th[2]
        Hwe = np.array([[np.cos(tita), -np.sin(tita), loc_x_y_th[0]],
                        [np.sin(tita), np.cos(tita), loc_x_y_th[1]],
                        [0, 0, 1]])

        Hec = np.array([[1, 0, descentre],
                        [0, 1, 0],
                        [0, 0, 1]])

        extremos = np.array(
            [trasera_izda, delantera_izda, delantera_dcha, trasera_dcha, trasera_izda, frontal_robot, trasera_dcha])
        robot = np.dot(Hwe, np.dot(Hec, np.transpose(extremos)))

        self.current_ax.plot(robot[0, :], robot[1, :], robotPlotStyle)

        return True

    def drawMapWithRobotLocations(self,
                                  robotPosVectors=[[0, 0, 0], [600, 600, 3.14]]):
        """ Overloaded version of drawMap to include robot positions """
        return self.drawMap(robotPosVectors=robotPosVectors)

    def drawMap(self, robotPosVectors=None):
        """
        Generates a plot with currently loaded map status

        NOTE:
        if verbose, it displays the plot
        if saveSnapshot: saves a figure as mapstatus_currenttimestamp_FIGNUM.png
        """
        # reset
        plt.gcf().clear()
        self.current_ax = plt.gca()

        self._drawGrid()

        # if flag is true, draw also current CostMatrix
        self._drawCostMatrix()

        if robotPosVectors:
            for loc in robotPosVectors:
                print("Robot in pos: ", loc)
                self._drawRobot(loc_x_y_th=loc, robotPlotStyle='b--')
            # plot last robot position with solid green line
            self._drawRobot(loc_x_y_th=loc, robotPlotStyle='g-')

        if Cfg.logMap:
            snapshot_name = Cfg.FOLDER_IMAGES + Cfg.logMap
            os.makedirs(os.path.dirname(snapshot_name), exist_ok=True)
            print("saving %s " % snapshot_name)
            plt.savefig(snapshot_name)

        # draw
        plt.gcf().canvas.draw()
        plt.gcf().canvas.flush_events()

        return plt.gcf()

    def findPath(self, point_ini, point_end):
        """ overloaded call to planPath (x_ini,  y_ini, x_end, y_end) """
        return self.planPath(point_ini[0], point_ini[1],
                             point_end[0], point_end[1])

    def closeAll(self):
        """
        Closes all graphs
        """
        plt.close('all')

    # ############################################################
    # METHODS to IMPLEMENT in P4
    # ############################################################

    def fillCostMatrix(self, x_end, y_end):
        """
        Fills the internal costMatrix variable where (x_end, y_end) will have cost 0, using the connection matrix
        :param x_end: horizontal cell coordinate of the cell with cost 0
        :param y_end: vertical cell coordinate of the cell with cost 0
        """
        USE_DIAGONALS = False  # if true use 8-neighbour, if false use 4-neighbour

        # first reset the cost matrix
        self._initCostMatrix()

        wavefront = [
            ((x_end, y_end), 0)]  # create the wavefront list points ((x,y),cost), and initialize with the end point
        while len(wavefront) != 0:  # evaluate each wavefront
            (x, y), c = wavefront.pop(
                0)  # get and remove the first element (will always have min-or-equal cost from all the others)
            if self.costMatrix[
                x, y] < 0 and 0 <= x < self.sizeX and 0 <= y < self.sizeY:  # inside the matrix and with uninitialized cost (because if already initialized it will always be better)
                self.costMatrix[x, y] = c  # set cost
                for neighbor in range(0, 8,
                                      1 if USE_DIAGONALS else 2):  # and for all neighbors (diagonal neighbors are odd)
                    if self.isConnected(x, y, neighbor):  # if they are connected
                        wavefront.append((self._neighbour(x, y, neighbor), c + 1))  # add to wavefront as cost+1

    def planPath(self, x_ini, y_ini, x_end, y_end):
        """
        Plans a path into the map from (x_ini, y_ini) to (x_end, y_end)
        :param x_ini: horizontal cell coordinate of the starting cell
        :param y_ini: vertical cell coordinate of the starting cell
        :param x_end: horizontal cell coordinate of the ending cell
        :param y_end: vertical cell coordinate of the ending cell
        :return: the list of cells which describe the calculated path from (x_ini, y_ini) to (x_end, y_end)
        """
        USE_DIAGONALS = False  # if true use 8-neighbour, if false use 4-neighbour

        # first, calculate costs
        self.fillCostMatrix(x_end, y_end)

        currentPath = [[x_ini, y_ini]]
        while [x_end, y_end] != currentPath[-1]:
            # Search the next neighbour
            best_neight = min([
                self._neighbour(*currentPath[-1], dir)  # search all neighbors
                for dir in range(0, 8, 1 if USE_DIAGONALS else 2)  # only horizontal coordinates
                if self.isConnected(*currentPath[-1], dir)  # which are connected
            ],
                key=lambda x: self.costMatrix[x[0], x[1]],  # and get the minimum cost
                default=None  # or none if not found
            )

            # add to path
            if best_neight is not None:
                currentPath.append(best_neight)
            else:
                # no path found
                raise Exception("No path found")

            if len(currentPath) > 1000:
                raise Exception("Infinite loop")

        if Cfg.plotMap:
            self.drawMap()

        # Make sure self.currentPath is a 2D numpy array
        self.currentPath = np.array(currentPath)
        return self.currentPath

    def replanPath(self, x_ini, y_ini, x_end, y_end):
        """
        Just invokes planPath (same arguments)
        """
        return self.planPath(x_ini, y_ini, x_end, y_end)

    def getNeighbour(self, angle):
        """
        Calculates the neighbour which corresponds to the given orientation value in angle
        E.g., when angle equals pi/2 rad (90º), returns 0
              when angle equals -pi/2 rad (-90º), returns 4
        7     0       1
        6   (x,y)     2
        5     4       3
        :param angle: orientation over the neigbour's circle
        :return: returns the corresponding neighbour to the given angle orientation
        """
        return ((-angle + np.pi / 8) // (np.pi / 4) + 2) % 8
