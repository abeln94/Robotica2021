import matplotlib.pyplot as plt

from functions.dibrobot import dibrobot

PADDING = 500


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
        self.lims = [[-PADDING, PADDING], [-PADDING, PADDING]]

    def update(self, loc):
        """ Adds a new localization to the map and updates it """
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

        # update limits in a smooth way (and using a very convoluted but compact code, can you guess what it does?)
        for i, f in ((0, plt.xlim), (1, plt.ylim)):
            lims = f()
            for j, d in ((0, -PADDING), (1, PADDING)):
                self.lims[i][j] = self.lims[i][j] * 0.8 + (lims[j] + d) * 0.2
            f(*self.lims[i])
        plt.gca().set_aspect(True)

        # draw
        plt.gcf().canvas.draw()
        plt.gcf().canvas.flush_events()

    def display(self, x, y):
        """ Show a full list of coordinates on the map """
        plt.grid()
        plt.plot(x, y, 'red')
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
