import sys
from argparse import ArgumentParser

# robot python version 3.5.3


# Fixed parameters
ROBOT_L = 117  # distance between robot wheels
ROBOT_r = 26.7  # radius of robots wheels
LIN_VEL = 150  # stable velocity for linear motion (mm/s)


class __CFG:
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
