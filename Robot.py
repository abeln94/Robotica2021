import numpy as np

from DeltaVal import DeltaVal
from dibrobot import dibrobot

try:
    import brickpi3  # import the BrickPi3 drivers
except ModuleNotFoundError:
    import simbrickpi3 as brickpi3
# tambien se podria utilizar el paquete de threading
from multiprocessing import Process, Value, Lock

import matplotlib.pyplot as plt
from time import perf_counter, sleep


class Robot:
    r = 22  # mm
    L = 121  # mm

    def __init__(self, init_position=None):
        """
        Initialize basic robot params.
        Initialize Motors and Sensors according to the set up in your robot
        """
        if init_position is None:
            init_position = [0.0, 0.0, 0.0]

        ######## UNCOMMENT and FILL UP all you think is necessary (following the suggested scheme) ########

        # Robot construction parameters
        # self.R = ??
        # self.L = ??
        # self. ...

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
        self.p = None

        # odometry shared memory values
        self.x = Value('d', 0.0)
        self.y = Value('d', 0.0)
        self.th = Value('d', 0.0)
        self.finished = Value('b', 1)  # boolean to show if odometry updates are finished

        # if we want to block several instructions to be run together, we may want to use an explicit Lock
        self.lock_odometry = Lock()
        # with self.lock_odometry:
        #   print('hello world', i)

        # odometry update period --> UPDATE value!
        self.P = 0.25  # 0.1 - 0.5

    def setSpeed(self, v, w):
        """ To be filled - These is all dummy sample code """
        print(f"setting speed to {v:.2f} {w:.2f}")

        # compute the speed that should be set in each motor ...
        wd = v / Robot.r + w * Robot.L / 2 / Robot.r
        wi = v / Robot.r - w * Robot.L / 2 / Robot.r

        # speedPower = 100
        # BP.set_motor_power(BP.PORT_B + BP.PORT_C, speedPower)

        self.BP.set_motor_dps(self.BP.PORT_B, np.rad2deg(wi))
        self.BP.set_motor_dps(self.BP.PORT_C, np.rad2deg(wd))

    def readSpeed(self):
        """ To be filled"""

        return 0, 0

    def readOdometry(self):
        """ Returns current value of odometry estimation """
        return self.x.value, self.y.value, self.th.value

    def startOdometry(self):
        """ This starts a new process/thread that will be updating the odometry periodically """
        self.finished.value = False
        self.p = Process(target=self.updateOdometry, args=())  # additional_params?))
        self.p.start()
        print("PID: ", self.p.pid)

    # You may want to pass additional shared variables besides the odometry values and stop flag
    def updateOdometry(self):  # , additional_params?):
        """ To be filled ...  """
        plt.ion()
        plt.figure('Robot simulation')
        plt.plot([], [])
        xpos = [0]
        ypos = [0]
        lims = [-500, 500]
        plt.gca().set_aspect(True)
        plt.show(block=False)

        leftMotor = self.BP.PORT_B
        rightMotor = self.BP.PORT_C

        updateTime = DeltaVal()
        pos = np.array([0.0, 0.0, 0.0])

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
            # dL = left.update(np.deg2rad(dL))
            # dR = right.update(np.deg2rad(dR))

            # wL = np.deg2rad(dL) / dT
            # wR = np.deg2rad(dR) / dT
            # v = Robot.r * (wL + wR) / 2
            # w = Robot.r * (wR - wL) / Robot.L
            # pos = simubot([v, w], pos, dT)

            sR = np.deg2rad(dR) * Robot.r  # distancia recorrida (mm) rueda derecha
            sL = np.deg2rad(dL) * Robot.r  # distancia recorrida (mm) rueda izquierda

            ds = (sL + sR) / 2
            dth = (sR - sL) / Robot.L
            pos[0] += ds * np.cos(pos[2] + dth / 2)
            pos[1] += ds * np.sin(pos[2] + dth / 2)
            pos[2] += dth

            # print(v, w)
            xpos.append(pos[0])
            ypos.append(pos[1])

            plt.gcf().clear()
            dibrobot(pos, 'blue')
            plt.plot(xpos, ypos, 'red')
            for i in range(2):
                if pos[i] - 500 < lims[0]:
                    lims[0] = lims[0] * 0.9 + (pos[i] - 500) * 0.1
                if pos[i] + 500 > lims[1]:
                    lims[1] = lims[1] * 0.9 + (pos[i] + 500) * 0.1
            plt.xlim(lims[0], lims[1])
            plt.ylim(lims[0], lims[1])
            plt.grid()
            plt.gcf().canvas.draw()
            plt.gcf().canvas.flush_events()

            ######## UPDATE FROM HERE with your code (following the suggested scheme) ########
            # print(f"Dummy update of odometry ...., X={self.x.value:.2f}, Y={self.y.value:.2f}, th={self.th.value:.2f}")
            # print("Dummy update of odometry ...., X=  %.2f" %(self.x.value) )

            # update odometry uses values that require mutex
            # (they are declared as value, so lock is implicitly done for atomic operations, BUT =+ is NOT atomic)

            # # Operations like += which involve a read and write are not atomic.
            # with self.x.get_lock():
            #     self.x.value += 1
            #
            # # to "lock" a whole set of operations, we can use a "mutex"
            # with self.lock_odometry:
            #     # self.x.value+=1
            #     self.y.value += 1
            #     self.th.value += 1

            # save LOG
            # Need to decide when to store a log with the updated odometry ...

            ######## UPDATE UNTIL HERE with your code ########

            tEnd = perf_counter()
            sleep(self.P - (tEnd - tIni))

        # print("Stopping odometry ... X= %d" %(self.x.value))
        print(f"Stopping odometry ... X={self.x.value:.2f}, Y={self.y.value:.2f}, th={self.th.value:.2f} \n")

    def stopOdometry(self):
        """ Stop the odometry thread """
        self.finished.value = True
        self.BP.reset_all()
