import time

import numpy as np

import Cfg
from classes.Position import Position
from classes.Robot import Robot
from functions.functions import norm_pi

robot = None


class Path:
    WORLD = 'W'
    ROBOT = 'R'
    DESTINATION = 'D'

    def __init__(self, robot):
        self.robot = robot
        self.idealPos = Position.FromLoc(robot.readOdometry(), Path.WORLD, Path.ROBOT)

    def move(self, x, y, local=True):

        if Cfg.noOdometry:
            currentPos = self.idealPos
        else:
            currentPos = Position.FromLoc(self.robot.readOdometry(), Path.WORLD, Path.ROBOT)

        if local:
            # parameters are local, get global
            lx, ly = x, y
            gx, gy, _ = (currentPos * Position.FromXYRad(x, y, 0, Path.ROBOT, Path.DESTINATION)).getLoc()

        else:
            # parameters are global, get local
            lx, ly, _ = (~currentPos * Position.FromXYRad(x, y, 0, Path.WORLD, Path.DESTINATION)).getLoc()
            gx, gy = x, y

        print("Moving to x=", gx, "y=", gy, "from", currentPos)

        if abs(ly) < 1e-6:
            # no rotation, advance only

            w = 0
            v = Cfg.MOTOR_VEL * Cfg.ROBOT_r * np.sign(lx)
            t = lx / v

        else:
            # both rotation and movement
            angle = np.arctan2(2 * lx * ly, lx ** 2 - ly ** 2)
            while angle < 0: angle += 2 * np.pi  # force a forward movement (prefer doing a 270º forward instead of 90º backward
            R = (lx ** 2 + ly ** 2) / (2 * ly)
            arc = angle * R

            # calculate velocity
            w = Cfg.MOTOR_VEL * Cfg.ROBOT_r / max(R + Cfg.ROBOT_L / 2, R - Cfg.ROBOT_L / 2, key=abs)
            v = w * R
            t = arc / v

        # act and wait
        self.robot.setSpeed(v, w)
        if Cfg.noOdometry:
            time.sleep(abs(t))
        else:
            waitUntil(x=gx, y=gy)

        # update ideal
        self.idealPos = self.idealPos.move(v, w, t)
        return self

    def rotate(self, th, local=True):
        if Cfg.noOdometry:
            currentPos = self.idealPos
        else:
            currentPos = Position.FromLoc(self.robot.readOdometry(), Path.WORLD, Path.ROBOT)

        if local:
            # parameters are local, get global
            lth = th
            _, _, gth = (currentPos * Position.FromXYRad(0, 0, th, Path.ROBOT, Path.DESTINATION)).getLoc()

        else:
            # parameters are global, get local
            _, _, lth = (~currentPos * Position.FromXYRad(0, 0, th, Path.WORLD, Path.DESTINATION)).getLoc()
            gth = th

        print("Rotating to th=", th, "from", currentPos)

        # velocity
        v = 0
        w = Cfg.MOTOR_VEL * Cfg.ROBOT_r / (Cfg.ROBOT_L / 2) * np.sign(lth)
        t = lth / w

        # act and wait
        self.robot.setSpeed(v, w)
        if Cfg.noOdometry:
            time.sleep(t)
        else:
            waitUntil(th=gth)

        # update ideal
        self.idealPos = self.idealPos.move(v, w, t)
        return self

    def gotoXY(self, x, y):
        return self.move(x, y, local=False)

    def advance(self, dist):
        return self.move(dist, 0)

    def arc(self, R, angle):
        if abs(R) < 1e-6:
            # rotation only
            return self.rotate(angle)
        else:
            # movement involved
            x = np.sin(angle) * R
            y = (1 - np.cos(angle)) * R
            return self.move(x, y)

    def turnLeftDeg(self, R, angle):
        return self.arc(R, np.deg2rad(angle))

    def turnRightDeg(self, R, angle):
        return self.arc(-R, np.deg2rad(angle))


def pause(sec=5):
    robot.setSpeed(0, 0)
    time.sleep(sec)


def doSnake():
    for i in range(10):
        robot.setSpeed(200, np.deg2rad(180))
        time.sleep(1)
        robot.setSpeed(200, -np.deg2rad(180))
        time.sleep(1)


def do180(d):
    Path(robot) \
        .advance(d) \
        .arc(0, 180) \
        .advance(d) \
        .arc(0, 180)


def do8(d):
    Path(robot) \
        .turnLeftDeg(d, 180) \
        .turnRightDeg(d, 180) \
        .turnRightDeg(d, 180) \
        .turnLeftDeg(d, 180)


def doBicy(d, a, r):
    alpha = np.rad2deg(np.arctan2(d - a, r))

    Path(robot). \
        turnRightDeg(a, 90 - alpha). \
        advance(r). \
        turnRightDeg(d, 180 + 2 * alpha). \
        advance(r). \
        turnRightDeg(a, 90 - alpha)


def waitUntil(x=None, y=None, th=None, r=100, angle=np.pi / 16):
    if x is None or y is None:
        # disable xy check
        x = y = 0
        r = -1
    if th is None:
        # disable th check
        th = 0
        angle = -1

    minr = np.inf
    minAngle = 2 * np.pi
    while True:
        # read
        rx, ry, rth = robot.readOdometry()
        rr = np.linalg.norm([x - rx, y - ry])
        rangle = abs(norm_pi(rth - th))

        # check
        if (r < 0 or (rr > minr and minr <= r)) and (angle < 0 or (rangle > minAngle and minAngle <= angle)):
            # If radius check is disabled the test passes
            # else if we are now farther and the previous (closer) value was inside the required radius, the test passes
            # Same for angle
            # if both test pass, then it is time to stop
            break

        # still nothing, update
        minr = rr
        minAngle = rangle


##################################################
###################### start #####################
##################################################
if __name__ == "__main__":

    try:
        # Instantiate Odometry. Default value will be 0,0,0
        # robot = Robot(init_position=args.pos_ini)
        robot = Robot()
        robot.startOdometry()

        # wait before start
        pause(3)
        # doSnake()
        # pause()

        # do the 180 trajectory
        if Cfg.length > 0:
            do180(Cfg.length)
            pause()

        # do the 8 trajectory
        if Cfg.radioD > 0 and Cfg.radioA < 0 and Cfg.distR < 0:
            do8(Cfg.radioD)
            pause()

        # do the bicy trajectory
        if Cfg.radioD > 0 and Cfg.radioA > 0 and Cfg.distR > 0:
            doBicy(Cfg.radioD, Cfg.radioA, Cfg.distR)
            pause()

    finally:
        # wrap up and close stuff before exiting
        if robot is not None: robot.stopOdometry()
