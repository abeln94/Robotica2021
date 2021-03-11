from typing import Union

import numpy as np

from functions.dibrobot import dibrobot
from functions.functions import hom, loc
from functions.simubot import simubot


class Position:
    """
    Represents a position from a reference
    """

    def __init__(self, T, referenceLabel: str, thisLabel: str):
        """
        Constructor from the homography matrix
        :param T: homography matrix
        :param referenceLabel: name of the reference point
        :param thisLabel: name of this point
        """
        self.thisLabel = thisLabel
        self.referenceLabel = referenceLabel
        self.T = T

    @classmethod
    def FromLoc(cls, loc: Union[np.ndarray, tuple], referenceLabel: str, thisLabel: str):
        """
        Constructor from the location array
        :param loc: location array [x,y,rad]
        :param referenceLabel: name of the reference point
        :param thisLabel: name of this point
        :return: The Position object
        """
        loc = np.array(loc)
        return cls(hom(loc), referenceLabel, thisLabel)

    @classmethod
    def FromXYRad(cls, x: float, y: float, angle: float, referenceLabel: str, thisLabel: str):
        """
        Constructor from position and angle in radians
        :param x: x position
        :param y: y position
        :param angle: angle (in radians)
        :param referenceLabel: name of the reference point
        :param thisLabel: name of this point
        :return: The Position object
        """
        return cls.FromLoc(np.array([x, y, angle]), referenceLabel, thisLabel)

    @classmethod
    def FromXYDeg(cls, x: float, y: float, angle: float, referenceLabel: str, thisLabel: str):
        """
        Constructor from position and angle in degrees
        :param x: x position
        :param y: y position
        :param angle: angle (in degrees)
        :param referenceLabel: name of the reference point
        :param thisLabel: name of this point
        :return: The Position object
        """
        return cls.FromXYRad(x, y, np.deg2rad(angle), referenceLabel, thisLabel)

    @classmethod
    def Zero(cls, referenceLabel: str, thisLabel: str = None):
        """
        Constructor for [0,0,0]
        :param label: name of the reference and this point
        :return: The Position object
        """
        if thisLabel is None: thisLabel = referenceLabel
        return cls.FromXYRad(0, 0, 0, referenceLabel, thisLabel)

    def getLoc(self) -> np.ndarray:
        """
        :return: the location array [x,y,rad]
        """
        return loc(self.T)

    def move(self, linVel: float, angVel: float, time: float, newLabel=None):
        """
        Moves the current position with the given velocity and time
        :param linVel: linear velocity
        :param angVel: angular velocity
        :param time: time of movement
        :param newLabel: name of the new point (unset for keep)
        :return: The Position object
        """
        return Position(
            hom(simubot([linVel, angVel], self.getLoc(), time)),
            self.referenceLabel,
            self.thisLabel if newLabel is None else newLabel
        )

    def draw(self, color='blue', size='g') -> None:
        """
        Draws the current position
        :param color: drawing color, default blue
        :param size: drawing size, default small
        """
        dibrobot(loc(self.T), color, size)

    def __mul__(self, other):
        """
        Multiplies two positions
        Use as P * Q
        Checks for correct references
        """
        assert self.thisLabel == other.referenceLabel
        return Position(self.T @ other.T, self.referenceLabel, other.thisLabel)

    def __invert__(self):
        """
        Inverts the position (reference point from this point)
        Use as ~P
        """
        return Position(np.linalg.inv(self.T), self.thisLabel, self.referenceLabel)

    def __str__(self) -> str:
        """
        toString
        """
        x, y, angle = loc(self.T)
        return "{}x{} = [x={:+.4f}, y={:+.4f}, th={:+.4f} ({:+.4f}º)]".format(self.referenceLabel, self.thisLabel, x, y, angle, np.rad2deg(angle))
