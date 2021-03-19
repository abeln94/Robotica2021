"""
The Robot main class
"""
import os
import time
# tambien se podria utilizar el paquete de threading
from multiprocessing import Process, Value, RLock

import numpy as np

import Cfg
from classes.DeltaVal import DeltaVal
from classes.Map import Map
from functions.functions import norm_pi
from functions.simubot import simubot
from functions.get_color_blobs import get_color_blobs

try:
    import brickpi3  # import the BrickPi3 drivers
except:
    import classes.simbrickpi3 as brickpi3

Cfg.add_argument("-f", "--log", help="Log odometry into a file", default=False)
Cfg.add_argument("-u", "--updatePeriod", help="Update period in seconds", type=float, default=0.1)
Cfg.add_argument("-e", "--exact", help="Use the exact method for odometry", action="store_true")
Cfg.add_argument("-p", "--plot", help="Show a plot with the values", action="store_true")


class Robot:
    def __init__(self, init_position=None):
        """
        Initialize basic robot params.
        Initialize Motors and Sensors according to the set up in your robot
        """
        if init_position is None:
            init_position = [0.0, 0.0, 0.0]

        ##################################################
        # Motors and sensors setup

        # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.
        self.BP = brickpi3.BrickPi3()

        # Configure sensors, for example a touch sensor.
        # self.BP.set_sensor_type(self.BP.PORT_1, self.BP.SENSOR_TYPE.TOUCH)

        # reset encoder B and C (or all the motors you are using)
        self.leftMotor = self.BP.PORT_B
        self.rightMotor = self.BP.PORT_C
        self.BP.offset_motor_encoder(self.leftMotor, self.BP.get_motor_encoder(self.leftMotor))
        self.BP.offset_motor_encoder(self.rightMotor, self.BP.get_motor_encoder(self.rightMotor))

        ##################################################        # Odometry

        self.p = None  # the odometry process

        # if we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_odometry = RLock()

        # odometry shared memory values
        self.x = Value('d', init_position[0], lock=self.lock_odometry)
        self.y = Value('d', init_position[1], lock=self.lock_odometry)
        self.th = Value('d', init_position[2], lock=self.lock_odometry)
        self.finished = Value('b', True, lock=self.lock_odometry)  # boolean to show if odometry updates are finished

    def setSpeed(self, v, w):
        """ 
        Sets the speed of the robot to v linear motion (mm/s) and w angular motion (rad/s) 
        :param v: linear velocity
        :param w: angular velocity
        """
        print("Setting speed to {:.2f} {:.2f}".format(v, w))

        # compute the speed that should be set in each motor ...
        wd = v / Cfg.ROBOT_r + w * Cfg.ROBOT_L / 2 / Cfg.ROBOT_r
        wi = v / Cfg.ROBOT_r - w * Cfg.ROBOT_L / 2 / Cfg.ROBOT_r

        self.BP.set_motor_dps(self.BP.PORT_B, np.rad2deg(wi))
        self.BP.set_motor_dps(self.BP.PORT_C, np.rad2deg(wd))

    def readSpeed(self, readTime=0.1):
        """
        Reads the robot speed
        :param readTime: time to check in seconds (the bigger the better but also the longer)
        :return: the velocity as tuple (v,w) v linear velocity mm/s , w angular velocity rad/s
        """

        # read
        previ = self.BP.get_motor_encoder(self.leftMotor)
        prevd = self.BP.get_motor_encoder(self.rightMotor)

        # wait
        time.sleep(readTime)

        # read
        posti = self.BP.get_motor_encoder(self.leftMotor)
        postd = self.BP.get_motor_encoder(self.rightMotor)

        # calculate
        wi = np.deg2rad(posti - previ) / readTime
        wd = np.deg2rad(postd - prevd) / readTime

        v = (wi + wd) * Cfg.ROBOT_r / 2
        w = (wd - wi) * Cfg.ROBOT_r / Cfg.ROBOT_L

        return v, w

    def readOdometry(self):
        """ Returns current value of odometry estimation """
        with self.lock_odometry:
            return self.x.value, self.y.value, self.th.value

    def startOdometry(self):
        """ This starts a new process/thread that will be updating the odometry periodically """
        self.finished.value = False
        self.p = Process(target=self.updateOdometry)
        self.p.start()
        print("PID: ", self.p.pid)

    def updateOdometry(self):
        """ The odometry update process """

        # init map
        if Cfg.plot:
            map = Map()
            map.update(self.readOdometry())

        # init variables
        x, y, th = self.readOdometry()
        leftEncoder = DeltaVal(self.BP.get_motor_encoder(self.leftMotor))
        rightEncoder = DeltaVal(self.BP.get_motor_encoder(self.rightMotor))

        if Cfg.log:
            fileName = Cfg.FOLDER_LOGS + Cfg.log
            os.makedirs(os.path.dirname(fileName), exist_ok=True)
            logFile = open(fileName, "w")
            logFile.write("Timestamp, X, Y, Theta\n")

        # loop
        while not self.finished.value:
            # current processor time in a floating point value, in seconds
            tIni = time.clock()

            # get values
            dL = leftEncoder.update(self.BP.get_motor_encoder(self.leftMotor))
            dR = rightEncoder.update(self.BP.get_motor_encoder(self.rightMotor))

            # compute updates
            if Cfg.exact:
                # exact, long
                dT = 1  # any value will work
                wL = np.deg2rad(dL) / dT
                wR = np.deg2rad(dR) / dT

                v = Cfg.ROBOT_r * (wL + wR) / 2
                w = Cfg.ROBOT_r * (wR - wL) / Cfg.ROBOT_L
                x, y, th = simubot([v, w], np.array([x, y, th]), dT)

            else:
                # inexact, fast
                sR = np.deg2rad(dR) * Cfg.ROBOT_r
                sL = np.deg2rad(dL) * Cfg.ROBOT_r

                ds = (sL + sR) / 2
                dth = (sR - sL) / Cfg.ROBOT_L
                x += ds * np.cos(th + dth / 2)
                y += ds * np.sin(th + dth / 2)
                th = norm_pi(th + dth)

            # update
            with self.lock_odometry:
                self.x.value = x
                self.y.value = y
                self.th.value = th

            # display
            print("Updated odometry ... X={:.2f}, Y={:.2f}, th={:.2f} ({:.2f}º)".format(x, y, th, np.rad2deg(th)))

            if Cfg.plot:
                map.update([x, y, th])

            # save LOG
            if Cfg.log:
                logFile.write("{}, {}, {}, {}\n".format(tIni, x, y, th))

            ######## UPDATE UNTIL HERE with your code ########

            # wait for next update
            tEnd = time.clock()
            secs = Cfg.updatePeriod - (tEnd - tIni)
            if secs > 0: time.sleep(secs)

        print("Stopping odometry ... X={:.2f}, Y={:.2f}, th={:.2f} ({:.2f}º)".format(x, y, th, np.rad2deg(th)))
        if Cfg.log:
            logFile.close()

    def stopOdometry(self):
        """ Stop the odometry thread """
        self.finished.value = True
        self.BP.reset_all()

    def trackObject(self, colorRangeMin=(0, 0, 0), colorRangeMax=(255, 255, 255), targetPosition=(0.6, 0.8), allowedPositionError=0.1):
        """
        Track one object with indicated color until the target size and centroid are reached
        :param colorRangeMin: minimum BGR color value taken into consideration for blob's detection
        :param colorRangeMax: maximum BGR color value taken into consideration for blob's detection
        :param targetPosition: on image target coordinates value of the blob's centroid
        :param allowerPositionError: error value allowed in the centroid measures to consider the target has been reached
        """
        targetPositionReached = False
        # 0. Parameters
        ANGULAR_SPEED = np.deg2rad(20)
        LINEAR_SPEED = 50
        MOVEMENT_TIME = 0.1 # seconds
        ANGULAR_SPEED_LOST = np.deg2rad(45) # angular speed when no blob found
        # 1. Loop running the tracking until target (centroid position and size) reached
        while not targetPositionReached:
            # 1.1. search the most promising blob ..
            position = get_blob(colorRangeMin, colorRangeMax)
            # 1.2. check the given position
            if position is not None:
                # 1.3 blob found, check its position for planning movement
                x, y = position
                if (x - targetPosition[0]) <= allowedPositionError and (y - targetPosition[1]) <= allowedPositionError:
                    # 1.4 target position reached, let's catch the ball
                    targetPositionReached = True
                else:
                    # 1.4 angular movement to get a proper orientation to the target
                    angular_speed = 0
                    if x < targetPosition[0]:
                        angular_speed = -ANGULAR_SPEED # turn right
                    elif x > targetPosition[0]:
                        angular_speed = ANGULAR_SPEED  # turn left
                    # 1.5 linear movement to get closer the target
                    linear_speed = 0
                    if y < targetPosition[1]:
                        # robot is too close, it has to go back
                        linear_speed = -LINEAR_SPEED
                    elif y > targetPosition[1]:
                        # ball is far, robot has to approach it
                        linear_speed = LINEAR_SPEED
                    self.setSpeed(linear_speed, angular_speed)
                    time.sleep(MOVEMENT_TIME)
                    self.setSpeed(0, 0)
            else:
                # 1.3 no blob found, turn around until finding something similar to the target
                self.setSpeed(0, ANGULAR_SPEED_LOST)
                while get_blob(colorRangeMin, colorRangeMax) == None:
                    time.sleep(MOVEMENT_TIME)
                self.setSpeed(0, 0)
        # 2. Then catch the ball
        self.catch()

    def catch(self):
        """ Implements the closing of the robot's claw """

