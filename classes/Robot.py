"""
The Robot main class
"""
import time
# tambien se podria utilizar el paquete de threading
from multiprocessing import Process, Value, RLock

import numpy as np

import Cfg
from classes.DeltaVal import DeltaVal
from classes.Map import Map
from functions.functions import norm_pi
from functions.perf_counter_exact import perf_counter_exact
from functions.simubot import simubot

try:
    import brickpi3  # import the BrickPi3 drivers
except:
    import classes.simbrickpi3 as brickpi3


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
            logFile = open("./logs/" + Cfg.log, "w")
            logFile.write("Timestamp, X, Y, Theta\n")

        # loop
        while not self.finished.value:
            # current processor time in a floating point value, in seconds
            tIni = perf_counter_exact()

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
            tEnd = perf_counter_exact()
            secs = Cfg.updatePeriod - (tEnd - tIni)
            if secs > 0: time.sleep(secs)

        print("Stopping odometry ... X={:.2f}, Y={:.2f}, th={:.2f} ({:.2f}º)".format(x, y, th, np.rad2deg(th)))
        if Cfg.log:
            logFile.close()

    def stopOdometry(self):
        """ Stop the odometry thread """
        self.finished.value = True
        self.BP.reset_all()
