# tambien se podria utilizar el paquete de threading
from multiprocessing import Process, Value, RLock
from time import perf_counter, sleep

import numpy as np

import Cfg
from classes.DeltaVal import DeltaVal
from classes.Map import Map
from functions.simubot import simubot

try:
    import brickpi3  # import the BrickPi3 drivers
except ModuleNotFoundError:
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
        self.BP.set_sensor_type(self.BP.PORT_1, self.BP.SENSOR_TYPE.TOUCH)

        # reset encoder B and C (or all the motors you are using)
        self.BP.offset_motor_encoder(self.BP.PORT_B, self.BP.get_motor_encoder(self.BP.PORT_B))
        self.BP.offset_motor_encoder(self.BP.PORT_C, self.BP.get_motor_encoder(self.BP.PORT_C))

        ##################################################        # Odometry

        self.p = None  # the odometry process

        # if we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_odometry = RLock()

        # odometry shared memory values
        self.x = Value('d', init_position[0], lock=self.lock_odometry)
        self.y = Value('d', init_position[1], lock=self.lock_odometry)
        self.th = Value('d', init_position[2], lock=self.lock_odometry)
        self.finished = Value('b', True, lock=self.lock_odometry)  # boolean to show if odometry updates are finished

        # odometry update period --> UPDATE value!
        self.P = 0.25  # 0.1 - 0.5

    def setSpeed(self, v, w):
        print("Setting speed to {:.2f} {:.2f}".format(v, w))

        # compute the speed that should be set in each motor ...
        wd = v / Cfg.ROBOT_r + w * Cfg.ROBOT_L / 2 / Cfg.ROBOT_r
        wi = v / Cfg.ROBOT_r - w * Cfg.ROBOT_L / 2 / Cfg.ROBOT_r

        self.BP.set_motor_dps(self.BP.PORT_B, np.rad2deg(wi))
        self.BP.set_motor_dps(self.BP.PORT_C, np.rad2deg(wd))

    def readSpeed(self):
        """ To be filled"""
        return 0, 0

    def readOdometry(self):
        """ Returns current value of odometry estimation """
        with self.lock_odometry:
            return self.x.value, self.y.value, self.th.value

    def startOdometry(self):
        """ This starts a new process/thread that will be updating the odometry periodically """
        self.finished.value = False
        self.p = Process(target=self.updateOdometry, args=())  # additional_params?))
        self.p.start()
        print("PID: ", self.p.pid)

    # You may want to pass additional shared variables besides the odometry values and stop flag
    def updateOdometry(self):  # , additional_params?):

        if not Cfg.noPlot:
            map = Map()
            map.update(self.readOdometry())

        leftMotor = self.BP.PORT_B
        rightMotor = self.BP.PORT_C

        if Cfg.exact:
            updateTime = DeltaVal()

        logFile = open("./logs/odometry.log", "w")
        logFile.write("Timestamp; X; Y; Theta\n")

        while not self.finished.value:
            # current processor time in a floating point value, in seconds
            tIni = perf_counter()

            # compute updates
            if Cfg.exact:
                dT = updateTime.update(perf_counter())

            # get values
            dL = self.BP.get_motor_encoder(leftMotor)
            dR = self.BP.get_motor_encoder(rightMotor)
            self.BP.offset_motor_encoder(leftMotor, dL)
            self.BP.offset_motor_encoder(rightMotor, dR)

            if Cfg.exact:
                # exact, long
                wL = np.deg2rad(dL) / dT
                wR = np.deg2rad(dR) / dT
                v = Cfg.ROBOT_r * (wL + wR) / 2
                w = Cfg.ROBOT_r * (wR - wL) / Cfg.ROBOT_L
                [x, y, th] = simubot([v, w], np.array(self.readOdometry()), dT)

                with self.lock_odometry:
                    self.x.value = x
                    self.y.value = y
                    self.th.value = th
            else:
                # inexact, fast
                sR = np.deg2rad(dR) * Cfg.ROBOT_r
                sL = np.deg2rad(dL) * Cfg.ROBOT_r

                ds = (sL + sR) / 2
                dth = (sR - sL) / Cfg.ROBOT_L
                th = self.th.value
                dx = ds * np.cos(th + dth / 2)
                dy = ds * np.sin(th + dth / 2)

                with self.lock_odometry:
                    self.x.value += dx
                    self.y.value += dy
                    self.th.value += dth

            # display
            x, y, th = self.readOdometry()
            print("Updated odometry ... X={:.2f}, Y={:.2f}, th={:.2f} ({:.2f}º)".format(x, y, th, np.rad2deg(th)))

            if not Cfg.noPlot:
                map.update(self.readOdometry())

            # save LOG
            logFile.write("{:.2f}; {:.2f}; {:.2f}; {:.2f}\n".format(tIni - perf_counter(), *self.readOdometry()))
            # Need to decide when to store a log with the updated odometry ...

            ######## UPDATE UNTIL HERE with your code ########

            if not Cfg.noWait:
                tEnd = perf_counter()
                secs = self.P - (tEnd - tIni)
                if secs > 0: sleep(secs)

        print("Stopping odometry ... X={:.2f}, Y={:.2f}, th={:.2f}".format(*self.readOdometry()))
        logFile.close()

    def stopOdometry(self):
        """ Stop the odometry thread """
        self.finished.value = True
        self.BP.reset_all()
