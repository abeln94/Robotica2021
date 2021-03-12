import time

import numpy as np

import Cfg
from classes import Map
from classes.Robot import Robot
from functions.functions import norm_pi

robot = None


def pause(sec=5):
    robot.setSpeed(0, 0)
    time.sleep(sec)


def doSnake():
    for i in range(10):
        robot.setSpeed(200, np.deg2rad(180))
        time.sleep(1)
        robot.setSpeed(200, -np.deg2rad(180))
        time.sleep(1)


def do180(d, v):
    # linear movement
    robot.setSpeed(v, 0)
    if Cfg.noOdometry:
        time.sleep(d / v)
    else:
        waitUntil(x=d, y=0)

    # half circle
    robot.setSpeed(0, np.deg2rad(45))
    if Cfg.noOdometry:
        time.sleep(4)
    else:
        waitUntil(th=np.pi)

    # linear movement (back to origin)
    robot.setSpeed(v, 0)
    if Cfg.noOdometry:
        time.sleep(d / v)
    else:
        waitUntil(x=0, y=0)

    # half circle (back to orientation 0)
    robot.setSpeed(0, np.deg2rad(45))
    if Cfg.noOdometry:
        time.sleep(4)
    else:
        waitUntil(th=0)


def do8(d, v):
    # first half circle
    robot.setSpeed(v, v / d)
    if Cfg.noOdometry:
        time.sleep(np.pi * d / v)
    else:
        waitUntil(x=0, y=2 * d)

    # seconds full circle
    robot.setSpeed(v, -v / d)
    if Cfg.noOdometry:
        time.sleep(2 * np.pi * d / v)
    else:
        waitUntil(x=0, y=4 * d)
        waitUntil(x=0, y=2 * d)

    # third half circle
    robot.setSpeed(v, v / d)
    if Cfg.noOdometry:
        time.sleep(np.pi * d / v)
    else:
        waitUntil(x=0, y=0)


def doBicy(d, a, r, v):
    alpha = np.arctan2(d - a, r)
    c = np.cos(alpha)
    s = np.sin(alpha)

    # first quarter circle
    robot.setSpeed(v, -v / a)
    if Cfg.noOdometry:
        time.sleep((np.pi / 2 - alpha) * a / v)
    else:
        waitUntil(x=c * a, y=-(1 - s) * a)

    # linear motion
    robot.setSpeed(v, 0)
    if Cfg.noOdometry:
        time.sleep(r / v)
    else:
        waitUntil(x=c * a + s * r, y=-(1 - s) * a - c * r)

    # half circle
    robot.setSpeed(v, -v / d)
    if Cfg.noOdometry:
        time.sleep((np.pi + 2 * alpha) * d / v)
    else:
        waitUntil(x=-c * a - s * r, y=-(1 - s) * a - c * r)

    # linear motion again
    robot.setSpeed(v, 0)
    if Cfg.noOdometry:
        time.sleep(r / v)
    else:
        waitUntil(x=-c * a, y=-(1 - s) * a)

    # last quarter circle
    robot.setSpeed(v, -v / a)
    if Cfg.noOdometry:
        time.sleep((np.pi / 2 - alpha) * a / v)
    else:
        waitUntil(x=0, y=0)


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
        robot = Robot([Map.GRID / 2, Map.GRID / 2, np.deg2rad(90)])
        robot.startOdometry()

        # wait before start
        pause(3)
        # doSnake()
        # pause()

        # do the 180 trajectory
        if Cfg.length > 0:
            do180(Cfg.length, Cfg.LIN_VEL)
            pause()

        # do the 8 trajectory
        if Cfg.radioD > 0 and Cfg.radioA < 0 and Cfg.distR < 0:
            do8(Cfg.radioD, Cfg.LIN_VEL)
            pause()

        # do the bicy trajectory
        if Cfg.radioD > 0 and Cfg.radioA > 0 and Cfg.distR > 0:
            doBicy(Cfg.radioD, Cfg.radioA, Cfg.distR, Cfg.LIN_VEL)
            pause()

    finally:
        # wrap up and close stuff before exiting
        if robot is not None: robot.stopOdometry()
