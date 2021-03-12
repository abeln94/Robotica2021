import time
from argparse import ArgumentParser

import numpy

from classes.Map import Map

"""
This script does a plot of a given log file, allowing just plotting or doing an animation of the movement through the 
map based on the timestamps of the records
"""

parser = ArgumentParser()

parser.add_argument("-f", "--file", help="LogFile to plot", type=str, required=True)
parser.add_argument("-i", "--image", help="Outputs an image file", type=str, default=None)
parser.add_argument("-a", "--animation", help="Animates the robot movement", action="store_true")

args = parser.parse_args()

if __name__ == "__main__":
    # prepare
    data = numpy.genfromtxt("./logs/" + args.file, delimiter=',', names=True)
    map = Map()

    if args.animation:
        # animate
        for t, *xyth in data:
            map.update(xyth)
            delay = t - time.clock()
            if delay > 0: time.sleep(delay)
    else:
        # display directly
        map.drawPath(data['X'], data['Y'])

    # save image
    if args.image:
        map.save("./" + args.image)

    # wait
    map.block()
