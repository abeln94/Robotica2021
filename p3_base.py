import Cfg
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

        # 2. Loop running the tracking until target (centroid position and size) reached
        #    Then catch the ball
        # TO-DO: ADD to the Robot class a method to track an object, given certain parameters
        # for example the different target properties we want (size, position, color, ..)
        # or a boolean to indicate if we want the robot to catch the object or not
        # At least COLOR, the rest are up to you, but always put a default value.
        res = False
        while res == False:
                res = robot.trackObject([0,0,0], [255,255,255], 500, [50,50], 20, 20)
                if res:
                    robot.catch()

        # 3. wrap up and close stuff ...
        # This currently unconfigure the sensors, disable the motors,
        # and restore the LED to the control of the BrickPi3 firmware.
        robot.stopOdometry()

    finally:
        # wrap up and close stuff before exiting
        if robot is not None: robot.stopOdometry()
