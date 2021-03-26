from classes import Cfg
from classes.Robot import Robot

# command line arguments
Cfg.add_argument("-c", "--color", help="color of the ball to track", type=float, default=40.0)

robot = None

#

if __name__ == "__main__":
    try:
        # Initialize Odometry. Default value will be 0,0,0
        robot = Robot()

        # 1. launch updateOdometry thread()
        robot.startOdometry()

        # 2. Look for and catch the ball
        robot.trackObject()

        # 3. wrap up and close stuff ...
        # This currently unconfigure the sensors, disable the motors,
        # and restore the LED to the control of the BrickPi3 firmware.
        robot.stopOdometry()

    finally:
        # wrap up and close stuff before exiting
        if robot is not None: robot.stopOdometry()
