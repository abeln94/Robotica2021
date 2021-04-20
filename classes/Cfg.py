import sys
from argparse import ArgumentParser

import numpy as np

# robot python version 3.5.3
# robot OpenCV version 3.4.13

# fixed configuration
FOLDER_LOGS = "./logs/"
FOLDER_IMAGES = FOLDER_LOGS + "pictures/"
FOLDER_MAPS = "./maps/"

# Fixed parameters
ROBOT_L = 117  # distance between robot wheels
ROBOT_r = 26.7  # radius of robots wheels
LIGHT_OFFSET = 95  # x offset of the light sensor
LIN_VEL = 150  # stable velocity for linear motion (mm/s)
ANG_VEL = np.deg2rad(30)  # stable velocity for angular motion (rad/s)
CAMERA_WIDTH = 320  # camera width to obtain (px)
CAMERA_HEIGHT = 240  # camera height to obtain (px)


class __CFG:
    """
    Allows to have different arguments depending on which file is imported.
    In the header use
    Cfg.add_argument(...) // same parameters as parser.add_argument()

    Then simply use
    Cfg.parameter
    """

    def __init__(self, wrapped):
        self.__setattrs__(wrapped)
        self.__parser = ArgumentParser()  # global parser

    def add_argument(self, *args, **kwargs):
        """ Adds a new argument. Same usage as ArgumentParser.add_argument """
        if self.__parser is None:
            raise AttributeError("Already parsed")
        else:
            self.__parser.add_argument(*args, **kwargs)

    def __setattrs__(self, obj):
        """ setattr foreach """
        for key, value in vars(obj).items():
            setattr(self, key, value)

    def __getattr__(self, name):
        """ parses first time """
        if self.__parser is not None:
            # not parsed, parse and try again
            self.__setattrs__(self.__parser.parse_args())
            self.__parser = None
            return getattr(self, name)
        else:
            # bad argument
            raise AttributeError("Cfg doesn't contains " + name)


sys.modules[__name__] = __CFG(sys.modules[__name__])
