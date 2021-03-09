import matplotlib.pyplot as plt
import matplotlib.ticker as plticker

import Cfg
from functions.dibrobot import dibrobot

PADDING = 500  # padding of the plot
loc = plticker.MultipleLocator(base=Cfg.GRID)


class Map:
    """
    Shows a map of the robot position and its trail
    """

    def __init__(self):
        """ Initializes the map window """
        plt.ion()
        plt.figure('Robot simulation')
        plt.plot([], [])
        plt.show(block=False)

        self.xpos = [None]
        self.ypos = [None]

    def update(self, loc):
        """ Adds a new localization to the map and updates it """
        # reset
        plt.gcf().clear()

        # dib trail
        if loc[0] != self.xpos[-1] or loc[1] != self.ypos[-1]:
            self.xpos.append(loc[0])
            self.ypos.append(loc[1])
        self.drawPath(self.xpos, self.ypos)

        # draw robot
        dibrobot(loc, 'blue')

        # draw
        plt.gcf().canvas.draw()
        plt.gcf().canvas.flush_events()

    def drawPath(self, x, y):
        """ Show a full list of coordinates on the map """
        # display grid
        plt.gca().xaxis.set_major_locator(loc)
        plt.gca().yaxis.set_major_locator(loc)
        plt.grid()

        # display path
        plt.plot(x, y, 'red')

        # add padding to limits
        for f in (plt.xlim, plt.ylim):
            lims = f()
            f(lims[0] - PADDING, lims[1] + PADDING)
        plt.gca().set_aspect(True)

    def save(self, path):
        """ Saves the currently displayed map to a file """
        plt.savefig(path)

    def block(self):
        """ Blocks the map until the user closes it """
        plt.show(block=True)
