import time
from argparse import ArgumentParser

import numpy as np

from Robot import Robot

# get and parse arguments passed to main
# Add as many args as you need ...
parser = ArgumentParser()
parser.add_argument("-d", "--radioD", help="Radio to perform the 8-trajectory (mm)", type=float, default=40.0)

robot = None

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

        print(f"X value at the beginning from main X={robot.x.value}")

        # 1. launch updateOdometry Process()
        robot.startOdometry()

        robot.setSpeed(200, 0)
        time.sleep(5)
        robot.setSpeed(0, np.deg2rad(45))
        time.sleep(3)
        robot.setSpeed(200, 0)
        time.sleep(5)
        robot.setSpeed(0, 0)

        # 2. perform trajectory
        # d = 400
        # T = 200
        # robot.setSpeed(T, T / d)
        # time.sleep(np.pi * d / T)
        #
        # robot.setSpeed(T, -T / d)
        # time.sleep(2 * np.pi * d / T)
        #
        # robot.setSpeed(T, T / d)
        # time.sleep(np.pi * d / T)
        #
        # robot.setSpeed(0, 0)
        time.sleep(3)

        # with robot.lock_odometry:
        #     print(f"Odom values at main at the END: {robot.x.value:.2f}, {robot.y.value:.2f}, {robot.th.value:.2f}")

        # PART 1:
        # robot.setSpeed()
        # until ...

        # PART 2:
        # robot.setSpeed()
        # until ...

        # ...

    finally:
        # 3. wrap up and close stuff ...
        # This currently unconfigure the sensors, disable the motors,
        # and restore the LED to the control of the BrickPi3 firmware.

        # even if the program gets interrupted by Ctrl+C on the keyboard.
        # THIS IS IMPORTANT if we want that motors STOP when we Ctrl+C ...
        if robot is not None: robot.stopOdometry()
