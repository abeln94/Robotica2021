import matplotlib.pyplot as plt
import numpy

from functions.dibrobot import dibrobot


class Map:
    """
    Shows a map of the robot position and its trail
    """

    def __init__(self, file = None):
        # init
        plt.ion()
        plt.figure('Robot simulation')
        plt.plot([], [])
        plt.gca().set_aspect(True)
        plt.show(block=False)

        self.xpos = [None]
        self.ypos = [None]
        self.lims = [-500, 500]
        self.file = file


    def update(self, loc):
        # reset
        plt.gcf().clear()
        plt.grid()

        # dib robot
        dibrobot(loc, 'blue')

        # dib trail
        if loc[0] != self.xpos[-1] or loc[1] != self.ypos[-1]:
            self.xpos.append(loc[0])
            self.ypos.append(loc[1])
        plt.plot(self.xpos, self.ypos, 'red')

        # update limits
        for i in range(2):
            if loc[i] - 500 < self.lims[0]:
                self.lims[0] = self.lims[0] * 0.9 + (loc[i] - 500) * 0.1
            if loc[i] + 500 > self.lims[1]:
                self.lims[1] = self.lims[1] * 0.9 + (loc[i] + 500) * 0.1
        plt.xlim(self.lims[0], self.lims[1])
        plt.ylim(self.lims[0], self.lims[1])

        # draw
        plt.gcf().canvas.draw()
        plt.gcf().canvas.flush_events()


    # Plots the log file passed in the constructor (if exists)
    def plotFile(self):
        if self.file:  
            data = numpy.genfromtxt("./logs/" + self.file, delimiter=',')
            for i in range(1, len(data)):
                self.update( [data[i][1], data[i][2], data[i][3]] )