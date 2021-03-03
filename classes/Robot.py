# tambien se podria utilizar el paquete de threading
from multiprocessing import Process, Value, RLock
from time import perf_counter, sleep

import numpy as np

from classes.DeltaVal import DeltaVal
from classes.Map import Map

try:
    import brickpi3  # import the BrickPi3 drivers
except ModuleNotFoundError:
    import classes.simbrickpi3 as brickpi3


class Robot:
    r = 28  # mm
    L = 121  # mm

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

        ##################################################
        # Odometry

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
        print(f"Setting speed to {v:.2f} {w:.2f}")

        # compute the speed that should be set in each motor ...
        wd = v / Robot.r + w * Robot.L / 2 / Robot.r
        wi = v / Robot.r - w * Robot.L / 2 / Robot.r

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

        map = Map()
        map.update(self.readOdometry())

        leftMotor = self.BP.PORT_B
        rightMotor = self.BP.PORT_C

        updateTime = DeltaVal()

        while not self.finished.value:
            # current processor time in a floating point value, in seconds
            tIni = perf_counter()

            # compute updates
            dT = updateTime.update(perf_counter())

            # simulation
            dL = self.BP.get_motor_encoder(leftMotor)
            dR = self.BP.get_motor_encoder(rightMotor)
            self.BP.offset_motor_encoder(leftMotor, dL)
            self.BP.offset_motor_encoder(rightMotor, dR)

            ############## exact, long
            # wL = np.deg2rad(dL) / dT
            # wR = np.deg2rad(dR) / dT
            # v = Robot.r * (wL + wR) / 2
            # w = Robot.r * (wR - wL) / Robot.L
            # [x, y, th] = simubot([v, w], np.array(self.readOdometry()), dT)
            #
            # with self.lock_odometry:
            #     self.x.value = x
            #     self.y.value = y
            #     self.th.value = th
            ############## /

            ############## inexact, fast
            sR = np.deg2rad(dR) * Robot.r
            sL = np.deg2rad(dL) * Robot.r

            ds = (sL + sR) / 2
            dth = np.arctan2((sR - sL), Robot.L)
            th = self.th.value
            dx = ds * np.cos(th + dth / 2)
            dy = ds * np.sin(th + dth / 2)

            with self.lock_odometry:
                self.x.value += dx
                self.y.value += dy
                self.th.value = th + dth
            ############## /

            map.update(self.readOdometry())

            # save LOG
            # Need to decide when to store a log with the updated odometry ...

            ######## UPDATE UNTIL HERE with your code ########

            tEnd = perf_counter()
            secs = self.P - (tEnd - tIni)
            if secs > 0: sleep(secs)

        # print("Stopping odometry ... X= %d" %(self.x.value))
        print(f"Stopping odometry ... X={self.x.value:.2f}, Y={self.y.value:.2f}, th={self.th.value:.2f} \n")

    def stopOdometry(self):
        """ Stop the odometry thread """
        self.finished.value = True
        self.BP.reset_all()
