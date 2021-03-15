import time

import numpy

import Cfg
from classes.Map import Map

"""
This script does a plot of a given log file, allowing just plotting or doing an animation of the movement through the 
map based on the timestamps of the records
"""

Cfg.add_argument("-f", "--file", help="LogFile to plot", type=str, required=True)
Cfg.add_argument("-i", "--image", help="Outputs an image file", type=str, default=None)
Cfg.add_argument("-a", "--animation", help="Animates the robot movement", action="store_true")

if __name__ == "__main__":
    Cfg.parse()

    # prepare
    data = numpy.genfromtxt("./logs/" + Cfg.file, delimiter=',', names=True)
    map = Map()

    if Cfg.animation:
        # animate
        for t, *xyth in data:
            map.update(xyth)
            delay = t - time.clock()
            if delay > 0: time.sleep(delay)
    else:
        # display directly
        map.drawPath(data['X'], data['Y'])

    # save image
    if Cfg.image:
        map.save("./" + Cfg.image)

    # wait
    map.block()
