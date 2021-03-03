import time
from argparse import ArgumentParser

import numpy as np

from classes.Robot import Robot
from functions.functions import norm_pi

# get and parse arguments passed to main
# Add as many args as you need ...
parser = ArgumentParser()
parser.add_argument("-d", "--radioD", help="Radio to perform the 8-trajectory (mm)", type=float, default=40.0)

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


def do180():
    robot.setSpeed(200, 0)
    # time.sleep(5)
    waitUntil(x=1000, y=0)

    robot.setSpeed(0, np.deg2rad(45))
    # time.sleep(4)
    waitUntil(th=np.pi)

    robot.setSpeed(200, 0)
    # time.sleep(5)
    waitUntil(x=0, y=0)


def do8(d=400, T=200):
    robot.setSpeed(T, T / d)
    time.sleep(np.pi * d / T)

    robot.setSpeed(T, -T / d)
    time.sleep(2 * np.pi * d / T)

    robot.setSpeed(T, T / d)
    time.sleep(np.pi * d / T)


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
        rx, ry, rth = robot.readOdometry()
        rr = np.linalg.norm([x - rx, y - ry])
        rangle = abs(norm_pi(rth - th))
        # check xy
        if rr <= minr:
            # we are closer, keep
            minr = rr
        else:
            # we are more far, check
            if minr <= r:
                # inside required, stop
                break
        # check angle
        if rangle <= minAngle:
            # we are closer, keep
            minAngle = rangle
        else:
            # we are more far, check
            if minAngle <= angle:
                # inside required, stop
                break
        time.sleep(0.1)


if __name__ == "__main__":
    args = parser.parse_args()

    if args.radioD < 0:
        print('d must be a positive value')
        exit(1)

    ##################################################
    ###################### start #####################
    ##################################################

    try:
        # Instantiate Odometry. Default value will be 0,0,0
        # robot = Robot(init_position=args.pos_ini)
        robot = Robot()
        robot.startOdometry()

        # doSnake()
        # pause()

        do180()
        pause()

        # do8()
        # pause()

    finally:
        # 3. wrap up and close stuff ...
        # This currently unconfigure the sensors, disable the motors,
        # and restore the LED to the control of the BrickPi3 firmware.

        # even if the program gets interrupted by Ctrl+C on the keyboard.
        # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        if robot is not None: robot.stopOdometry()
