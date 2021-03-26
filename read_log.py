import os
import time

import numpy

from classes import Cfg
from classes.Map import Map

"""
This script does a plot of a given log file, allowing just plotting or doing an animation of the movement through the 
map based on the timestamps of the records
"""

Cfg.add_argument("-f", "--file", help="LogFile to plot", type=str, required=True)
Cfg.add_argument("-i", "--image", help="Outputs an image file", type=str, default=None)
Cfg.add_argument("-a", "--animation", help="Animates the robot movement", action="store_true")

if __name__ == "__main__":

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
        fileName = Cfg.FOLDER_IMAGES + Cfg.image
        os.makedirs(os.path.dirname(fileName), exist_ok=True)
        map.save(fileName)

    # wait
    map.block()
